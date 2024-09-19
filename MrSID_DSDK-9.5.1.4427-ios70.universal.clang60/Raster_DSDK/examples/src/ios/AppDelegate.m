
#import "AppDelegate.h"
#import "ViewController.h"

@implementation AppDelegate

@synthesize window = _window;

- (void)dealloc
{
   [_window release];
   [super dealloc];
}

- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions
{
   // the working directory needs to be the parent of the data directory
   [[NSFileManager defaultManager] changeCurrentDirectoryPath:NSHomeDirectory()];

   _window = [[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]];

   ViewController *viewController =
         [[ViewController alloc] initWithStyle:UITableViewStyleGrouped];
   
   [_window setRootViewController:viewController];
   [viewController release];
   [_window makeKeyAndVisible];
   return YES;
}

@end
