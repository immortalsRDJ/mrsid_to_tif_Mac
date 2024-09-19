
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
         CASE(UserTutorial_iterator);
         CASE(UserTutorial_read);
         CASE(DumpMG4Info);
         CASE(IterateOverPoints);
         CASE(DecodeMG4ToTXT);
         CASE(DecodeMrSIDRaster);
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
#define CASE(tag) case TEST_##tag: tag(); _state[TEST_##tag] = TS_Passed; if(!runall) break 
         
      case TEST_AllTests:
         runall = true;

      CASE(UserTutorial_iterator);
      CASE(UserTutorial_read);
      CASE(DumpMG4Info);
      CASE(IterateOverPoints);
      CASE(DecodeMG4ToTXT);
      CASE(DecodeMrSIDRaster);
      CASE(UserTest);
         
#undef CASE
      default: break;
   }

   [tableView reloadData];
}

@end
