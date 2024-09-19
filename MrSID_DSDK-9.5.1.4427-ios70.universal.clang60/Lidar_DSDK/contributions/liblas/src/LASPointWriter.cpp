// Prevent conflicts between liblas's stdint.hpp and <stdint.h>
#define __STDC_CONSTANT_MACROS

//self
#include "lidar/LASPointWriter.h"
#include "lidar/formats_status.h"
#include "lidar/core_status.h"
#include "lidar/Version.h"
#include "lidar/FileIO.h"
#include "lidar/Error.h"
#include "lidar/Private.h"


#include "laswriter.hpp"
#include "ogr_spatialref.h"
#include "cpl_conv.h"
#include "geo_simpletags.h"
#include "gt_wkt_srs.h"

#include <time.h>

LT_USE_LIDAR_NAMESPACE

#define LASF_Projection            "LASF_Projection"



typedef void(*WriteFunc)(LASpoint *pt, lt_uint16 color,
size_t idx, const void *values, LASquantizer *q);
struct LASPointWriter::Handler
{
   WriteFunc writer;
   const void *data;
};

LASPointWriter::LASPointWriter(void) :
   m_output(NULL),
   m_fileVersion(VERSION_ANY),
   m_writeLAZ(false),
   m_header(NULL),
   m_writer(NULL),
   m_handler(NULL),
   m_numHandlers(0),
   m_total(0)
{
   memset (LASMax, 0, sizeof(LASMax));
   memset (LASMin, 0, sizeof(LASMax));
}

LASPointWriter::~LASPointWriter(void)
{
   if (m_writer != NULL)
      delete m_writer;
   if (m_header != NULL)
      delete m_header;
   if (m_lasinventory != NULL)
      delete m_lasinventory;
   DEALLOC(m_output);
}

IMPLEMENT_OBJECT_CREATE(LASPointWriter);

int
LASPointWriter::getRecordFormat(const PointInfo &pointInfo)
{
   int format = 0;
   if(pointInfo.hasChannel(CHANNEL_NAME_GPSTime_Week) ||
      pointInfo.hasChannel(CHANNEL_NAME_GPSTime_Adjusted))
      format += 1;

   if(pointInfo.hasChannel(CHANNEL_NAME_Red))
      format += 2;

   return format;
}

LASPointWriter::FileVersion
LASPointWriter::getFileVersion(const PointInfo &pointInfo)
{
   if(getRecordFormat(pointInfo) >= 2)
      return VERSION_1_2;
   else
      return VERSION_1_1;
}


void
LASPointWriter::init(const PointSource *src,
                     const char *path,
                     FileVersion fileVersion)
{
   assert(path != NULL);
   SimplePointWriter::init(src);
   m_output = STRDUP(path);
   m_fileVersion = fileVersion;
   m_lasinventory = new LASinventory();
}

void
LASPointWriter::setWriteLAZ(bool laz)
{
   m_writeLAZ = laz;
}

#define SIMPLE_WRITER(type, tag, func, cast, value, field) \
   static void writer_##func##_##tag(LASpoint *pt, lt_int16 /*color*/, \
   size_t idx, const type *values, LASquantizer *q) \
{ pt->field = static_cast<cast>(value); }

#define QUANTIZING_WRITER(type, tag, func, cast, value, field, getter) \
   static void writer_##func##_##tag(LASpoint *pt, lt_int16 /*color*/,\
   size_t idx, const type *values, LASquantizer *q) \
{ pt->field = q->getter(value); }

#define SIMPLE_COLOR_WRITER(type, tag, func, cast, value, field, icolor) \
   static void writer_##func##_##tag(LASpoint *pt, lt_int16 color, \
   size_t idx, const type *values, LASquantizer *q) \
{ pt->field[icolor] = static_cast<cast>(value); }

#define FLOAT_WRITER(func, cast, field) \
   SIMPLE_WRITER(float, FLOAT32, func, cast, values[idx], field) \
   SIMPLE_WRITER(double, FLOAT64, func, cast, values[idx], field)

#define QFLOAT_WRITER(func, cast, field, getter) \
   QUANTIZING_WRITER(float, FLOAT32, func, cast, values[idx], field, getter) \
   QUANTIZING_WRITER(double, FLOAT64, func, cast, values[idx], field, getter)

#define INT_WRITER(func, cast, field) \
   SIMPLE_WRITER(lt_int8, SINT8, func, cast, values[idx], field) \
   SIMPLE_WRITER(lt_uint8, UINT8, func, cast, values[idx], field) \
   SIMPLE_WRITER(lt_int16, SINT16, func, cast, values[idx], field) \
   SIMPLE_WRITER(lt_uint16, UINT16, func, cast, values[idx], field) \
   SIMPLE_WRITER(lt_int32, SINT32, func, cast, values[idx], field) \
   SIMPLE_WRITER(lt_uint32, UINT32, func, cast, values[idx], field) \
   SIMPLE_WRITER(lt_int64, SINT64, func, cast, values[idx], field) \
   SIMPLE_WRITER(lt_uint64, UINT64, func, cast, values[idx], field)

#define BOOL_WRITER(func, cast, field) \
   SIMPLE_WRITER(lt_int8, SINT8, func, cast, values[idx] ? 1 : 0, field) \
   SIMPLE_WRITER(lt_uint8, UINT8, func, cast, values[idx] ? 1 : 0, field) \
   SIMPLE_WRITER(lt_int16, SINT16, func, cast, values[idx] ? 1 : 0, field) \
   SIMPLE_WRITER(lt_uint16, UINT16, func, cast, values[idx] ? 1 : 0, field) \
   SIMPLE_WRITER(lt_int32, SINT32, func, cast, values[idx] ? 1 : 0, field) \
   SIMPLE_WRITER(lt_uint32, UINT32, func, cast, values[idx] ? 1 : 0, field) \
   SIMPLE_WRITER(lt_int64, SINT64, func, cast, values[idx] ? 1 : 0, field) \
   SIMPLE_WRITER(lt_uint64, UINT64, func, cast, values[idx] ? 1 : 0, field)

#define COLOR_WRITER(func, cast, field, icolor) \
   SIMPLE_COLOR_WRITER(lt_int8, SINT8, func, cast, values[idx], field, icolor) \
   SIMPLE_COLOR_WRITER(lt_uint8, UINT8, func, cast, values[idx], field, icolor) \
   SIMPLE_COLOR_WRITER(lt_int16, SINT16, func, cast, values[idx], field, icolor) \
   SIMPLE_COLOR_WRITER(lt_uint16, UINT16, func, cast, values[idx], field, icolor) \
   SIMPLE_COLOR_WRITER(lt_int32, SINT32, func, cast, values[idx], field, icolor) \
   SIMPLE_COLOR_WRITER(lt_uint32, UINT32, func, cast, values[idx], field, icolor) \
   SIMPLE_COLOR_WRITER(lt_int64, SINT64, func, cast, values[idx], field, icolor) \
   SIMPLE_COLOR_WRITER(lt_uint64, UINT64, func, cast, values[idx], field, icolor)

QFLOAT_WRITER(SetX, double, X, get_X)
QFLOAT_WRITER(SetY, double, Y, get_Y)
QFLOAT_WRITER(SetZ, double, Z, get_Z)
INT_WRITER(SetIntensity, lt_uint16, intensity)
INT_WRITER(SetReturnNumber, lt_uint16, return_number)
INT_WRITER(SetNumberOfReturns, lt_uint16, number_of_returns)
BOOL_WRITER(SetScanDirection, lt_uint64, scan_direction_flag)
BOOL_WRITER(SetFlightLineEdge, lt_uint64, edge_of_flight_line)
INT_WRITER(SetClassification, lt_uint8, classification)
INT_WRITER(SetScanAngleRank, lt_int8, scan_angle_rank)
INT_WRITER(SetUserData, lt_uint8, user_data)
INT_WRITER(SetPointSourceId, lt_uint16, point_source_ID)
FLOAT_WRITER(SetTime, double, gps_time)
COLOR_WRITER(SetRed, lt_uint16, rgb, 0)
COLOR_WRITER(SetGreen, lt_uint16, rgb, 1)
COLOR_WRITER(SetBlue, lt_uint16, rgb, 2)

static void add_vlr(LASheader *header,
                    const char *userId, U16 recordId,
                    const char *description,
                    size_t length, const void *data)
{
   if (length <= MAX_VALUE<U16>())
   {
      U8 *buffer = new U8[length];
      memcpy(buffer, data, length);
      header->add_vlr(userId, recordId, static_cast<U16>(length), buffer);
      LASvlr *vlr = &(header->vlrs[header->number_of_variable_length_records - 1]);
      if (description)
         strncpy(&(vlr->description[0]), description, sizeof(vlr->description));
      else
         strncpy(&(vlr->description[0]), "", sizeof(vlr->description));
   }
}


void
LASPointWriter::writeBegin(const PointInfo &pointInfo)
{
   m_total = 0;
   int recordFormat = getRecordFormat(pointInfo);
   int fileVersion = m_fileVersion;
   if(fileVersion == VERSION_ANY)
      fileVersion = getFileVersion(pointInfo);

   // writer should write out intersection of requested fields
   // and available fields
   
   m_header = new LASheader();
   sprintf(m_header->generating_software, "LizardTech %s", Version::getSDKVersionString());
   m_header->system_identifier[0] = '\0';
   time_t theTime = time(0);
   tm *t = gmtime(&theTime);
   // tm_yday is days since January 1 (0-365)
   // but LAS Spec says 1 January is day 1
   m_header->file_creation_day = static_cast<U16>(t->tm_yday + 1);
   m_header->file_creation_year = static_cast<U16>(t->tm_year + 1900);
   m_header->version_major = static_cast<U8>(fileVersion / 10);
   m_header->version_minor = static_cast<U8>(fileVersion % 10);
   m_header->point_data_format = static_cast<U8>(recordFormat);
   if (pointInfo.hasChannel(CHANNEL_NAME_GPSTime_Adjusted))
      m_header->global_encoding |= 0x1;
   if (m_header->version_minor > 2)
   {
      m_header->header_size += 8;
      m_header->offset_to_point_data += 8;
   }

   switch (m_header->point_data_format)
   {
   case 0:
      m_header->point_data_record_length = 20;
      break;
   case 1:
      m_header->point_data_record_length = 28;
      break;
   case 2:
      m_header->point_data_record_length = 26;
      break;
   case 3:
      m_header->point_data_record_length = 34;
      break;
   case 4:
      m_header->point_data_record_length = 57;
      break;
   case 5:
      m_header->point_data_record_length = 63;
      break;
   case 6:
      m_header->point_data_record_length = 30;
      break;
   case 7:
      m_header->point_data_record_length = 36;
      break;
   case 8:
      m_header->point_data_record_length = 38;
      break;
   case 9:
      m_header->point_data_record_length = 59;
      break;
   case 10:
      m_header->point_data_record_length = 67;
      break;
   default:
      break;
   }
   m_header->x_scale_factor = m_scale[0];
   m_header->y_scale_factor = m_scale[1];
   m_header->z_scale_factor = m_scale[2];
   m_header->x_offset = m_offsets[0];
   m_header->y_offset = m_offsets[1];
   m_header->z_offset = m_offsets[2];

   bool hasProjectionVLR = false;

   const Metadata &metadata = getMetadata();
   
   for(size_t idx = 0; idx < metadata.getNumRecords(); idx += 1)
   {
      const char *key;
      const char *description;
      MetadataDataType datatype;
      const void *value;
      size_t length;

      metadata.get(idx, key, description, datatype, value, length);

      if (!strcmp(METADATA_KEY_SystemID, key))
         strncpy(m_header->system_identifier, static_cast<const char *>(value), sizeof(m_header->system_identifier));
      else if (!strcmp(METADATA_KEY_ProjectID, key))
      {
         unsigned int data[11];
         ::sscanf(static_cast<const char *>(value),
                  "{%08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X}",
                  &data[0], &data[1], &data[2], &data[3], &data[4], &data[5],
                  &data[6], &data[7], &data[8], &data[9], &data[10]);

         m_header->project_ID_GUID_data_1 = data[0];
         m_header->project_ID_GUID_data_2 = data[1];
         m_header->project_ID_GUID_data_3 = data[2];
         m_header->project_ID_GUID_data_4[0] = data[3];
         m_header->project_ID_GUID_data_4[1] = data[4];
         m_header->project_ID_GUID_data_4[2] = data[5];
         m_header->project_ID_GUID_data_4[3] = data[6];
         m_header->project_ID_GUID_data_4[4] = data[7];
         m_header->project_ID_GUID_data_4[5] = data[8];
         m_header->project_ID_GUID_data_4[6] = data[9];
         m_header->project_ID_GUID_data_4[7] = data[10];
      }
      else if (!strcmp(METADATA_KEY_PointRecordsByReturnCount, key))
      {
         const double *p = static_cast<const double *>(value);
         for(int i = 0; i < 5; i += 1)
            m_header->number_of_points_by_return[i] = static_cast<lt_uint32>(p[i]);
      }
      else if (!strcmp(METADATA_KEY_FileSourceID, key))
      {
         int fileSourceId;
         sscanf(static_cast<const char *>(value), "%d", &fileSourceId);
         m_header->file_source_ID = static_cast<lt_uint16>(fileSourceId);
      }
      else if (!strcmp(METADATA_KEY_FileCreationDate, key))
      {
         // to do.
         //LASHeader_SetProjectId(m_header, static_cast<const char *>(value));
      }
      else if (strcmp(METADATA_KEY_GeneratingSoftware, key) == 0)
         // Always "LizardTech V-X.Y"
         continue;
      else if(strcmp(METADATA_KEY_LASBBox, key) == 0)
      {
         LASMin[0] = static_cast<const double*>(value)[0];
         LASMin[1] = static_cast<const double*>(value)[2];
         LASMin[2] = static_cast<const double*>(value)[4];
         LASMax[0] = static_cast<const double*>(value)[1];
         LASMax[1] = static_cast<const double*>(value)[3];
         LASMax[2] = static_cast<const double*>(value)[5];
         continue;
      }   
      else
      {
         const char *e = strstr(key, "::");
         if (e != NULL)
         {
         char userId[17];
         memset(userId, 0, sizeof(userId));
         memcpy(userId, key, MIN(e - key, 16));

         int recordId;
         sscanf(e + 2, "%d", &recordId);

            if (strcmp(LASF_Projection, userId) == 0)
         {
               if (recordId == 2111 || recordId == 2111 ||
                   recordId == 34735 || recordId == 34736 || recordId == 34737)
                  hasProjectionVLR = true;
         }

            add_vlr(m_header, userId, recordId, description, length, value);
      }
   }
   }

#if defined _DEBUG && 0
   DebugBreak();
#endif
   if (!hasProjectionVLR)
   {
      const char *wkt = getSrc()->getWKT();
      if (wkt != NULL && *wkt != '\0')
      {
         if (m_fileVersion > VERSION_1_3)
         {
            // should we use the 2111 (math transform) or 2112 (coordinate system)
            add_vlr(m_header, LASF_Projection, 2112, NULL, strlen(wkt) + 1, wkt);
   }
   else
   {
            ST_TIFF* tiff = ST_Create();
            GTIF* gtiff = GTIFNewSimpleTags(tiff);
            if (GTIFSetFromOGISDefn(gtiff, wkt) && GTIFWriteKeys(gtiff))
      {
               int count, type;
               void *buffer;

               if (ST_GetKey(tiff, 34735, &count, &type, &buffer))
               {
                  assert(type == STT_SHORT);
                  add_vlr(m_header, LASF_Projection, 34735, NULL, count * sizeof(U16), buffer);
               }

               if (ST_GetKey(tiff, 34736, &count, &type, &buffer))
               {
                  assert(type == STT_DOUBLE);
                  add_vlr(m_header, LASF_Projection, 34736, NULL, count * sizeof(double), buffer);
               }

               if (ST_GetKey(tiff, 34737, &count, &type, &buffer))
               {
                  assert(type == STT_ASCII);
                  add_vlr(m_header, LASF_Projection, 34737, NULL, count * sizeof(U8), buffer);
               }
            }

            GTIFFree(gtiff);
            ST_Destroy(tiff);
         }
      }
   }

   m_numHandlers = pointInfo.getNumChannels();
   m_handler = ALLOC(Handler, sizeof(Handler) * m_numHandlers);
   for(size_t i = 0; i < m_numHandlers; i += 1)
   {
#define CASE(func, type) \
   case DATATYPE_##type: \
      writer = reinterpret_cast<WriteFunc>(writer_##func##_##type); \
      break

#define HANDLE_FLOAT(tag, func) \
   if(::strcmp(name, CHANNEL_NAME_##tag) == 0) \
   { \
      switch(datatype) \
      { \
         CASE(func, FLOAT32); \
         CASE(func, FLOAT64); \
         default: \
            THROW_LIBRARY_ERROR(LTL_STATUS_FORMATS_INVALID_PARAM) \
               ("invalid floating point number provided"); \
      } \
   }

#define HANDLE_INT(tag, func) \
   if(::strcmp(name, CHANNEL_NAME_##tag) == 0) \
   { \
      switch(datatype) \
      { \
         CASE(func, SINT8); \
         CASE(func, UINT8); \
         CASE(func, SINT16); \
         CASE(func, UINT16); \
         CASE(func, SINT32); \
         CASE(func, UINT32); \
         CASE(func, SINT64); \
         CASE(func, UINT64); \
         default: \
            THROW_LIBRARY_ERROR(LTL_STATUS_FORMATS_INVALID_PARAM) \
               ("invalid integer provided"); \
      } \
   }

      const ChannelInfo &channelInfo = pointInfo.getChannel(i);
      const char *name = channelInfo.getName();
      DataType datatype = channelInfo.getDataType();
      WriteFunc writer = NULL;
      HANDLE_FLOAT(X, SetX)
      else HANDLE_FLOAT(Y, SetY)
      else HANDLE_FLOAT(Z, SetZ)
      else HANDLE_INT(Intensity, SetIntensity)
      else HANDLE_INT(ReturnNum, SetReturnNumber)
      else HANDLE_INT(NumReturns, SetNumberOfReturns)
      else HANDLE_INT(ScanDir, SetScanDirection)
      else HANDLE_INT(EdgeFlightLine, SetFlightLineEdge)
      else HANDLE_INT(ClassId, SetClassification)
      else HANDLE_INT(ScanAngle, SetScanAngleRank)
      else HANDLE_INT(UserData, SetUserData)
      else HANDLE_INT(SourceId, SetPointSourceId)
      else HANDLE_FLOAT(GPSTime_Week, SetTime)
      else HANDLE_FLOAT(GPSTime_Adjusted, SetTime)
      else HANDLE_INT(Red, SetRed)
      else HANDLE_INT(Green, SetGreen)
      else HANDLE_INT(Blue, SetBlue)
      else
         THROW_LIBRARY_ERROR(LTL_STATUS_FORMATS_INVALID_PARAM)
            ("invalid LAS field: %s", name);

      m_handler[i].writer = writer;
      m_handler[i].data = NULL;
   }

   LASwriteOpener opener;
   opener.set_file_name(m_output);

   if (m_writeLAZ)
      opener.set_format(LAS_TOOLS_FORMAT_LAZ);
   else
      opener.set_format(LAS_TOOLS_FORMAT_LAS);

   m_writer = opener.open(m_header);
   opener.set_file_name(0);

   if(m_writer == NULL)
   {
      THROW_OS_ERROR()
         ("Unable to create output %s.", m_output);
   }
}

void
LASPointWriter::writePoints(const PointData &points,
                            size_t numPoints,
                            ProgressDelegate *delegate)
{
   static const char message[] = "writing LAS file";
   if(delegate != NULL)
   {
      delegate->updateCompleted(0, message);
      if(delegate->getCancelled())
         THROW_LIBRARY_ERROR(LTL_STATUS_CORE_OPERATION_CANCELLED)
            ("LAS Write operation cancelled.");
   }

   m_total += numPoints;
   if (m_total > MAX_VALUE<lt_uint32>())
      THROW_LIBRARY_ERROR(LTL_STATUS_FORMATS_INVALID_PARAM)
      ("Attempt to write 2 ^ 32 or more points to a LAS file.");

   for(size_t i = 0; i < m_numHandlers; i += 1)
      m_handler[i].data = points.getChannel(i).getData();

   Handler *stop = m_handler + m_numHandlers;
   LASpoint pt;
   pt.init(m_header,
           m_header->point_data_format,
           m_header->point_data_record_length,
           m_header);
   for (size_t i = 0; i < numPoints; i += 1)
   {
      // marshal the various fields from LT SDK (in handler->data) into LASlib SDK (in pt)
      for(Handler *handler = m_handler; handler < stop; handler += 1)
         handler->writer(&pt, 0, i, handler->data, m_header);

      m_lasinventory->add(&pt);
      if (!m_writer->write_point(&pt))
         THROW_LIBRARY_ERROR(LTL_STATUS_FORMATS_LIBLAS_WRITER)
         ("Unable to write point");
   }

   if(delegate != NULL)
      delegate->updateCompleted(static_cast<double>(numPoints), message);
}

void
LASPointWriter::writeEnd(PointSource::count_type numPoints,
                         const Bounds &bounds)
{
   bool failed = numPoints == 0 && bounds == Bounds::Huge();
   if(m_writer != NULL)
   {
      if(!failed)
      {
         m_header->number_of_point_records = m_lasinventory->number_of_point_records;
         for (int i = 0; i < 5; i++) 
            m_header->number_of_points_by_return[i] = m_lasinventory->number_of_points_by_return[i + 1];
         m_header->max_x = m_header->get_x(m_lasinventory->max_X);
         m_header->min_x = m_header->get_x(m_lasinventory->min_X);
         m_header->max_y = m_header->get_y(m_lasinventory->max_Y);
         m_header->min_y = m_header->get_y(m_lasinventory->min_Y);
         m_header->max_z = m_header->get_z(m_lasinventory->max_Z);
         m_header->min_z = m_header->get_z(m_lasinventory->min_Z);

         if (! m_writer->update_header(m_header, FALSE, TRUE))
            THROW_LIBRARY_ERROR((LTL_STATUS_FORMATS_LIBLAS_WRITER))
            ("Unable to update header");
      }
      m_writer->close();
      if (m_header != NULL)
         delete m_header;
      m_header = NULL;
   }
   if(failed && m_output != NULL)
      FileIO::deleteFile(m_output); 
}
