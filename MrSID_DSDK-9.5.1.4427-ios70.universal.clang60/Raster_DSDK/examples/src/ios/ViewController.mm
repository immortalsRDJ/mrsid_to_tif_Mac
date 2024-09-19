
#import "ViewController.h"

#include "main.h"

@implementation ViewController

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
   // Return YES for supported orientations
   return YES;
}

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
   return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
   return NUM_TESTS;
}

- (UITableViewCell *)tableView:(UITableView *)tableView
         cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
   static NSString *CellId = @"Cell";
   
   UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:CellId];
   if (cell == nil)
   {
      cell = [[[UITableViewCell alloc] initWithStyle:UITableViewCellStyleValue1
                                     reuseIdentifier:CellId] autorelease];
   }

   NSString *n = nil;
   TestState ts = TS_NotRun;
   switch([indexPath row])
   {
#define CASE(tag) case TEST_##tag: n = @"Run " #tag; ts = _state[TEST_##tag]; break 
      
      CASE(AllTests);
      CASE(DecodeJP2ToBBB);
      CASE(DecodeJP2ToMemory);
      CASE(DecodeMrSIDToMemory);
      CASE(DecodeMrSIDLidar);
      CASE(DecodeMrSIDToRaw);
      CASE(DecodeNITFToBBB);
      CASE(DecodeMrSIDToTIFF);
      CASE(DecodeJP2ToJPG);
      CASE(DerivedImageFilter);  
      CASE(DerivedImageReader);
      CASE(DerivedImageWriter);
      CASE(DerivedStream);
      CASE(ErrorHandling);
      CASE(GeoScene);
      CASE(ImageInfo);
      CASE(InterruptDelegate);
      CASE(MetadataDump);
      CASE(Pipeline);
      CASE(ProgressDelegate);
      CASE(SceneBuffer);
      CASE(UsingCInterface);
      CASE(UsingCStream);
      CASE(UsingStreams);
      CASE(UserTest);

#undef CASE
      default: n = @""; break;
   }

   [[cell textLabel] setText:n];

   if(ts == TS_Passed)
      [[cell detailTextLabel] setText:@"passed"];
   else if(ts == TS_Failed)
      [[cell detailTextLabel] setText:@"FAILED"];
   else
      [[cell detailTextLabel] setText:@""];

   return cell;
}


#pragma mark - Table view delegate

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath
{
   
   bool runall = false;
   switch([indexPath row])
   {
#define CASE(tag) \
   case TEST_##tag: \
      _state[TEST_##tag] = LT_SUCCESS(tag()) ? TS_Passed : TS_Failed; \
      if(!runall) break
         
      case TEST_AllTests:
         runall = true;

      CASE(DecodeJP2ToBBB);
      CASE(DecodeJP2ToMemory);
      CASE(DecodeMrSIDToMemory);
      CASE(DecodeMrSIDLidar);
      CASE(DecodeMrSIDToRaw);
      CASE(DecodeNITFToBBB);
      CASE(DecodeMrSIDToTIFF);
      CASE(DecodeJP2ToJPG);
      CASE(DerivedImageFilter);  
      CASE(DerivedImageReader);
      CASE(DerivedImageWriter);
      CASE(DerivedStream);
      CASE(ErrorHandling);
      CASE(GeoScene);
      CASE(ImageInfo);
      CASE(InterruptDelegate);
      CASE(MetadataDump);
      CASE(Pipeline);
      CASE(ProgressDelegate);
      CASE(SceneBuffer);
      CASE(UsingCInterface);
      CASE(UsingCStream);
      CASE(UsingStreams);
      CASE(UserTest);
         
#undef CASE
      default: break;
   }

   [tableView reloadData];
}

@end
