#define STB_IMAGE_IMPLEMENTATION
#include <iostream>
#include <vector>
#include "stb_image.h"
#include "utils.hpp"
#include "DebugHelper.hpp"

int w,h,c;
int main()
{
  uint16_t *img1 = stbi_load_16("assets/T0.png",&w,&h,&c,0);
  uint16_t *img2 = stbi_load_16("assets/T1.png",&w,&h,&c,0);

  VertsFromDepth(img1, sourceVerts);
  VertsFromDepth(img2, destinationVerts);
  
  CalculateNormals(sourceVerts, sourceNormals);
  CalculateNormals(destinationVerts, destinationNormals);
  //WriteArrayToFile(sourceNormals, "sourceNormalGold.txt");

  //PrintArray(correspondenceVerts);
  //buildLinearSystem
  Align(2);
  cout<<"Min float value : "<<std::numeric_limits<float>::min()<<"\n";
  cout<<"Lowest float value : "<<std::numeric_limits<float>::lowest()<<"\n";
  cout<<"Max float value : "<<std::numeric_limits<float>::max()<<"\n";
  cout<<"NaN float value : "<<std::numeric_limits<float>::quiet_NaN()<<"\n";
}