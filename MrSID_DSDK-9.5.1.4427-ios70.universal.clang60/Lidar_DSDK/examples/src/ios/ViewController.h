
#import <UIKit/UIKit.h>


enum
{
   TEST_AllTests,
   
   TEST_UserTutorial_iterator,
   TEST_UserTutorial_read,
   
   TEST_DumpMG4Info,
   TEST_IterateOverPoints,
   TEST_DecodeMG4ToTXT,
   TEST_DecodeMrSIDRaster,
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
