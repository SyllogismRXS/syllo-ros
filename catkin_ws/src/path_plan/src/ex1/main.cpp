#include <iostream>

#include <path_plan/types.h>
#include <path_plan/a_star.h>

#include "SDL/SDL.h"
#include "SDL/SDL_image.h"

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <syllo_common/Filter.h>
#include <path_plan/a_star.h>

using std::cout;
using std::endl;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

bool wait = true;
void waitMouse()
{
     wait = true;
     while (wait) {
          cv::waitKey(1);          
     }
}

cv::Point mouse;
static void onMouse( int event, int x, int y, int flags, void* )
{
     if (event == CV_EVENT_LBUTTONDOWN ) {
          mouse.x = x;
          mouse.y = y;
          wait = false;
     }// else if (event == CV_EVENT_LBUTTONUP ) {        
}

int main(int argc, char *argv[])
{
     cout << "A* Path Planning Example." << endl;

     if (argc < 2) {
          cout << "usage: rosrun path_plan ex1 <image>.png" << endl;
          exit(-1);
     }

     //The images
     SDL_Surface* img = NULL;
     SDL_Surface* screen = NULL;

     SDL_Init( SDL_INIT_EVERYTHING );
     //screen = SDL_SetVideoMode( 640, 480, 32, SDL_SWSURFACE );

     //Load image
     img = IMG_Load(argv[1] );

     int height = img->h;
     int width = img->w;
     cout << "Image dimensions: " << img->h << " x " << img->w << endl;

     SDL_PixelFormat *fmt;
     SDL_Color *color;
     Uint8 index;

     //Create surface
     fmt = img->format;
 
     //Check the bitdepth of the surface */
     if(fmt->BitsPerPixel!=8){
          cout << "Not an 8-bit surface." << endl;
          exit(-2);
     }

     SDL_LockSurface( img );

     unsigned char* pixels;
     unsigned char* p;
     int rowstride, n_channels;
     int i,j;
     int k;
     double occ;
     int color_sum;
     double color_avg;
     bool negate = false;

     double free_th = 0.196;
     double occ_th = 0.65;

     std::vector<int> map;
     map.resize(img->h*img->w);

     // Get values that we'll need to iterate through the pixels
     rowstride = img->pitch;
     n_channels = img->format->BytesPerPixel;

     // Copy pixel data into the map structure
     pixels = (unsigned char*)(img->pixels);
     for(j = 0; j < img->h; j++)
     {
          for (i = 0; i < img->w; i++)
          {
               // Compute mean of RGB for this pixel
               p = pixels + j*rowstride + i*n_channels;
               color_sum = 0;
               for(k=0;k<n_channels;k++)
                    color_sum += *(p + (k));
               color_avg = color_sum / (double)n_channels;

               // If negate is true, we consider blacker pixels free, and whiter
               // pixels free.  Otherwise, it's vice versa.
               if(negate) {
                    occ = color_avg / 255.0;
               } else {
                    occ = (255 - color_avg) / 255.0;
               }
      
               // Apply thresholds to RGB means to determine occupancy values for
               // map.  Note that we invert the graphics-ordering of the pixels to
               // produce a map with cell (0,0) in the lower-left corner.
               if(occ > occ_th) {
                    //resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = +100;
                    map[MAP_IDX(img->w,i,img->h - j - 1)] = +100;
               } else if(occ < free_th) {
                    map[MAP_IDX(img->w,i,img->h - j - 1)] = 0;
               } else {
                    map[MAP_IDX(img->w,i,img->h - j - 1)] = -1;
               }
          }
     }

     SDL_UnlockSurface( img );

     ////Apply image to screen
     //SDL_BlitSurface( img, NULL, screen, NULL );
     //
     ////Update Screen
     //SDL_Flip( screen );
     //
     ////Pause
     //SDL_Delay( 2000 );
     
     //Free the loaded image
     SDL_FreeSurface( img );

     //Quit SDL
     SDL_Quit();
   
     syllo::AStar planner;
     syllo::Map syllo_map;

     cv::namedWindow("Map");
     cv::setMouseCallback( "Map", onMouse, 0 );
     
     do {           
          planner.reset();
   
          syllo_map.fill_map(map, width, height);
          syllo_map.set_origin(0,0);
          
          planner.set_map(&syllo_map);

          // Get user input about start and end states
          // Fill an OpenCV structure:
          cv::Mat cv_map_;
          cv_map_ = cv::Mat(height, width, CV_8UC1);     
          for (int y = 0; y < height; y++) {
               for (int x = 0; x < width; x++) {
                    double value = syllo_map.at(x,y);
                    value = normalize(value, 0, 100, 0, 255);
                    cv_map_.at<uchar>(height-1-y,x) = 255 - value;
               }
          }
                    
          cout << "Select initial state with mouse" << endl;
          cv::imshow("Map", cv_map_);
          //cv::waitKey();
          waitMouse();

          syllo::Node start(mouse.x,height-mouse.y);

          cout << "Select goal state with mouse" << endl;
          cv::imshow("Map", cv_map_);
          //cv::waitKey(1);
          waitMouse();
          
          syllo::Node goal(mouse.x, height-mouse.y);

          // Run the planner
          cout << "Planner running..." << endl;
          int status = planner.generate_path(start, goal);
          if (status != 0) {
               cout << "Failed to find path.\nError Code: " << status << endl;
               continue;
          }     
     
          // Get the resultant path
          std::list<syllo::Node*> path;
          path = planner.path();

          cout << "Path length: " << path.size() << endl;

          // Draw the path over the image
          std::list<syllo::Node*>::iterator it;
          for (it = path.begin(); it != path.end(); it++) {
               cv_map_.at<uchar>(height-1-(*it)->point().y,(*it)->point().x) = 90;
          }
               
          cout << "=============================" << endl;
          cout << "Hit any key to run again" << endl;
          cout << "Hit 'q' to (q)uit" << endl;
          cv::imshow("Map", cv_map_);
          int key = cv::waitKey(0);
          if (key == 'q') {
               break;
          }

     } while(true);

     return 0;
}
