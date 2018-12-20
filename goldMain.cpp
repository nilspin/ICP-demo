#define STB_IMAGE_IMPLEMENTATION
#include <iostream>
#include <vector>
#include <SDL2/SDL.h>
#include "stb_image.h"
#include "utils.hpp"
#include "DebugHelper.hpp"

int w,h,c;
uint iters;

void EventLoop();

int main()
{
  cout<<"Enter ICP iterations : ";
  std::cin>>iters;
  cout<<"\n";

  img1 = stbi_load_16("assets/0000.png",&w,&h,&c,0);
  img2 = stbi_load_16("assets/0000.png",&w,&h,&c,0);

  SetupCameraIntrinsic();
  VertsFromDepth(img1, sourceVerts);
  VertsFromDepth(img2, destinationVerts);
  
  //checkEquality(sourceVerts, destinationVerts);
  CalculateNormals(sourceVerts, sourceNormals);
  CalculateNormals(destinationVerts, destinationNormals);
  //checkEquality(sourceNormals, destinationNormals);
  //checkEquality(sourceVerts, destinationVerts);
  //Init SDL
  if(SDL_Init(SDL_INIT_VIDEO) < 0)  {exit(1);}
  window = SDL_CreateWindow("correspondence map", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w, h, SDL_WINDOW_SHOWN);
  surface = SDL_GetWindowSurface( window );

  //Fill the surface white
  SDL_FillRect( surface, NULL, SDL_MapRGB( surface->format, 0xFF, 0xFF, 0xFF ) );
  SDL_UpdateWindowSurface( window );

  Align(iters);

  //checkEquality(sourceNormals, destinationNormals);
  //checkEquality(sourceVerts, destinationVerts);
  EventLoop();
  
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