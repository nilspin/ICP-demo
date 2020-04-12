#define STB_IMAGE_IMPLEMENTATION
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <thread>
#include <SDL2/SDL.h>
#include "stb_image.h"
#include "utils.hpp"
#include "DebugHelper.hpp"
#include "common.h"

int w,h,c;

void EventLoop();

/*
TODO: Get following variables from cmd
* img1 img2
* correspondence threshold (ideally between 0.0-0.3)
* numPyramids
* pyramid iters
*/

std::thread align_thread;
int pyramid_size = 3;
float distThres = 0.2;
std::vector<int> pyramid_iters = {4,5,10};
int main(int argc, char* argv[])
{
	if (argc < 6) {
		usage(argv[0]);
		throw std::runtime_error("Incorrect arguments. Exiting.\n");
	}
	cout<<"There are "<<argc<<" arguments. They are : \n";
	for(int i=0;i<argc;++i)	{
		cout<<argv[i]<<"\n";
	}
  cout<<"\n";
  //img1 = stbi_load_16("assets/T1.png",&w,&h,&c,0);
  img1 = stbi_load_16(argv[1],&w,&h,&c,0);
  img2 = stbi_load_16(argv[2],&w,&h,&c,0);
  for(int argIter=3;argIter<argc;++argIter)	{
	if(std::string(argv[argIter])=="-corresThres"){
		distThres = std::stof(argv[argIter+1]);
		argIter++;
  	}
	if(std::string(argv[argIter])=="-pyramidLvls"){
		pyramid_size = std::stoi(argv[argIter+1]);
		argIter++;
	}
	if(std::string(argv[argIter])=="-pyramidIters"){
		pyramid_iters.clear();
		for(int pyItr = 0;pyItr<pyramid_size;++pyItr){
			pyramid_iters.emplace_back(std::stoi(argv[argIter+1]));
			argIter++;
		}
	}
  }

  std::vector<uint16_t> temp1(img1, img1+(640*480));
  std::vector<uint16_t> temp2(img2, img2+(640*480));
  sourceDepth = std::vector<float>(temp1.begin(), temp1.end());
  targetDepth = std::vector<float>(temp2.begin(), temp2.end());
  std::for_each(sourceDepth.begin(), sourceDepth.end(), [](float& a){ a = a/5000.0f;} );
  std::for_each(targetDepth.begin(), targetDepth.end(), [](float& a){ a = a/5000.0f;} );

  SetupCameraIntrinsic();
  VertsFromDepth(img1, sourceVerts);
  VertsFromDepth(img2, targetVerts);
  //WriteArrayToFile(sourceDepth,"sourceDepth.txt");
  //WriteArrayToFile(targetVerts,"targetVerts.txt");
  //WriteArrayToFile(sourceVerts,"sourceVerts.txt");
  //checkEquality(sourceVerts, targetVerts);
  //CalculateNormals(sourceVerts, sourceNormals);
  CalculateNormals(targetVerts, targetNormals);

  //checkEquality(sourceNormals, destinationNormals);
  //checkEquality(sourceVerts, destinationVerts);
  //Init SDL
  if(SDL_Init(SDL_INIT_VIDEO) < 0)  {exit(1);}
  window = SDL_CreateWindow("correspondence map", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w, h, SDL_WINDOW_SHOWN);
  surface = SDL_GetWindowSurface( window );

  //Fill the surface white
  SDL_FillRect( surface, NULL, SDL_MapRGB( surface->format, 0xFF, 0xFF, 0xFF ) );
  SDL_UpdateWindowSurface( window );

  Align();
  //align_thread = std::thread(Align,iters);
  //EventLoop();
  //align_thread.join();
  //checkEquality(sourceNormals, destinationNormals);
  //checkEquality(sourceVerts, destinationVerts);
  //EventLoop();

  SDL_DestroyWindow(window);
  SDL_Quit();
}

void EventLoop()  {

  bool quit = false;
  SDL_Event event;
  while(!quit)  {
      while (SDL_PollEvent(&event) != 0)  {
        switch (event.type) {
          case SDL_KEYDOWN:	//if Q windowkey is pressed then quit
            switch(event.key.keysym.sym)  {
              case SDLK_q :
                quit = true;
                break;
        }
        break;
      }
      break;
    }
  }
}
