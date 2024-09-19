
#import <UIKit/UIKit.h>


enum
{
   TEST_AllTests,
   TEST_DecodeJP2ToBBB,
   TEST_DecodeJP2ToMemory,
   TEST_DecodeMrSIDToMemory,
   TEST_DecodeMrSIDLidar,
   TEST_DecodeMrSIDToRaw,
   TEST_DecodeNITFToBBB,
   TEST_DecodeMrSIDToTIFF,
   TEST_DecodeJP2ToJPG,
   TEST_DerivedImageFilter,  
   TEST_DerivedImageReader,
   TEST_DerivedImageWriter,
   TEST_DerivedStream,
   TEST_ErrorHandling,
   TEST_GeoScene,
   TEST_ImageInfo,
   TEST_InterruptDelegate,
   TEST_MetadataDump,
   TEST_Pipeline,
   TEST_ProgressDelegate,
   TEST_SceneBuffer,
   TEST_UsingCInterface,
   TEST_UsingCStream,
   TEST_UsingStreams,
   TEST_UserTest,
   
   NUM_TESTS,
};

enum TestState
{
   TS_NotRun,
   TS_Passed,
   TS_Failed,
};

@interface ViewController : UITableViewController
{
   enum TestState _state[NUM_TESTS];
}


@end
