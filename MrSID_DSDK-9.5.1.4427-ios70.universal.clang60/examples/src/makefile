# $Id$

INCLUDE = \
   -I../../Lidar_DSDK/include \
   -I../../Raster_DSDK/include
   
SOURCE = \
   main.cpp \
   support.cpp \
   GetMrSIDFileVersion.cpp \
   UserTest.cpp

LIBRARIES = \
   -L../../Lidar_DSDK/lib -llti_lidar_dsdk \
   -L../../Raster_DSDK/lib -lltidsdk


OBJECTS = $(SOURCE:.cpp=.o)

all: examples

examples: $(OBJECTS)
	$(CXX) $(CCFLAGS) -o $@ $(OBJECTS) $(LIBRARIES) $(LDFLAGS)

.cpp.o:
	$(CXX) -c $(CCFLAGS) $(INCLUDE) -o $@ $<

clean:
	rm -f *.o
	rm -f examples


.PHONY: all clean

