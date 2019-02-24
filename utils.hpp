#ifndef UTILS_HPP
#define UTILS_HPP
#define GLM_ENABLE_EXPERIMENTAL

#include <vector>
#include <array>
#include <iostream>
#include <fstream>

#include "termcolor.hpp"
#include "DebugHelper.hpp"
#include "common.h"
#include "EigenUtil.h"
#include "colormap.h"
#include "Solver.h"
#include <SDL2/SDL.h>

uint numCorrPairs = 0;
uint16_t *img1 = nullptr;
uint16_t *img2 = nullptr;
mat3 K, K_inv;  //Camera intrinsic matrix, its inverse
mat3 Rot;
vec3 Trans;
Solver tracker;
static vector<float> sourceDepth(numCols*numRows);
static vector<float> targetDepth(numCols*numRows);
static vector<vec3> sourceVerts(numCols*numRows);
static vector<vec3> targetVerts(numCols*numRows);
static vector<vec3> targetNormals(numCols*numRows);
static vector<CoordPair> corrImageCoords;
//static vector<float> correspondenceWeights(numRows*numCols, 0.0f);
static vector<float> errorMask(numCols*numRows);
static vector<bool> mask(numCols*numRows, false);

static mat4 deltaT = mat4(1);
static double globalError = 0.0;

static SDL_Window *window = nullptr;
static SDL_Surface *surface = nullptr;

using namespace igl;

//declarations
void CreatePyramid(const std::vector<vec3>& src, std::vector<vector<vec3>>& target, int level);

void SetupCameraIntrinsic() {
  K = glm::make_mat3(intrinsics);
  K = transpose(K);
  K_inv = inverse(K);
}


void updateSurface()  {
  char* raw = reinterpret_cast<char*>(surface->pixels);
  auto minIt = std::min_element(errorMask.begin(), errorMask.end());
  auto maxIt = std::max_element(errorMask.begin(), errorMask.end());
  float min = *minIt;
  float max = *maxIt;
  float val;
  float r,g,b;
  for(int i=0;i<errorMask.size();++i)  {
    val = (errorMask[i] - min)/(max-min);
    char col = val*256;
    //uint w = sizeof()
    //uint16_t in = img1[i];
    //char d = in;
    //float d = correspondenceWeights[i]*0.05;
    if(mask[i]) {
      //colormap(COLOR_MAP_TYPE_VIRIDIS, val, r, g, b);
      //std::cout<<"r"<<r;
      //std::cout<<"g"<<g;
      //std::cout<<"b"<<b<<"\n";
      raw[4*i] = col; //b
      raw[(4*i)+1] = col; //g
      raw[(4*i)+2] = col; //r
      raw[(4*i)+3] = 255; //a
    }
  }
}

inline ivec2 cam2screenPos(vec3 p) {
  vec3 sp = K*p;
  //ivec2 spos = ivec2(sp.x, sp.y);
  ivec2 spos = ivec2(sp.x/sp.z + 0.5, sp.y/sp.z + 0.5);
  //ivec2 spos = ivec2(sp.x/sp.z, sp.y/sp.z);
  //std::cout<< glm::to_string(spos)  <<"\n";
  return spos;
}

void FindCorrespondences2(const vector<vec3>& srcVerts, const vector<vec3>& targetVerts, const vector<vec3>& targetNormals, const mat4& deltaT, float distThres, vector<CoordPair>& corrImageCoords)  {
  vec3 Trans, Scale, Skew;
  vec4 Perspective;
  quat RotQuat;
  mat3 Rot;
  glm::decompose(deltaT, Scale, RotQuat, Trans, Skew, Perspective);
  RotQuat = glm::conjugate(RotQuat);
  Rot = glm::toMat3(RotQuat);
  mat3 KRK_inv = K * Rot * K_inv;
  vec3 Kt = K * Trans;
  numCorrPairs = 0;

  for(uint v_s = 0; v_s < numRows; ++v_s)  {
    for(uint u_s = 0; u_s < numCols; ++u_s)  {
      uint index = v_s*numCols + u_s;
      vec3 imageCoord = vec3(u_s, v_s, 1.0);
      vec3 worldCoord = K_inv*imageCoord;
      vec3 pSrc = srcVerts[index];
      vec3 transPSrc = (Rot * pSrc) + Trans;//transform
      vec3 projected = K * (transPSrc);
      int u_t = (int)(projected.x / projected.z + 0.5);//non-homogenize
      int v_t = (int)(projected.y / projected.z + 0.5);
      if (u_t >= 0 && u_t < numCols && v_t >= 0 && v_t < numRows) {
        uint targetIndex = v_t*numCols + u_t;
        vec3 pTar = targetVerts[targetIndex];
        vec3 nTar = targetNormals[targetIndex];
        //square it
        //transformed_d_s = transformed_d_s*transformed_d_s;
        //d_t = d_t*d_t;
        vec3 diff = transPSrc - pTar;
        double d = glm::dot(diff, nTar);
        if ((d < distThres)) {
          numCorrPairs++;
          //std::cout<<numCorrPairs<<" - src: "<<glm::to_string(ivec2(u_s, v_s))<<" , target: "<<glm::to_string(ivec2(u_t, v_t))<<"\n";
          globalError += d;
          errorMask[index] = d;
          mask[index] = true;
          corrImageCoords.push_back(std::make_tuple(ivec2(u_s, v_s), ivec2(u_t, v_t), d));
        }
      }
    }
  }
}

void Align(uint iters)  {

  auto srcVerts_pyramid = vector<vector<vec3>>();
  auto targetVerts_pyramid = vector<vector<vec3>>();
  auto targetNormals_pyramid = vector<vector<vec3>>();
  CreatePyramid(sourceVerts, srcVerts_pyramid, pyramid_size);
  CreatePyramid(targetVerts, targetVerts_pyramid, pyramid_size);
  CreatePyramid(targetNormals, targetNormals_pyramid, pyramid_size);

  //std::cout<<"Pyramid size "<<srcVerts_pyramid.size()<<"\n";
  //int pyrSize = srcVerts_pyramid.size();
  //for(int i=0;i<pyrSize;++i) {
  //  std::cout<<"level"<<i<<" size "<<srcVerts_pyramid[i].size()<<"\n";
  //}

  for(uint level=0; level < pyramid_size; ++level) {

    std::cout<< "\n"<<termcolor::on_red<< "Iteration : "<< level << termcolor::reset << "\n";
    //ClearVector(correspondenceVerts);
    //ClearVector(correspondenceNormals);
    ClearVector(mask);
    ClearVector(errorMask);
    corrImageCoords.clear();
    //for(auto &i:System) {i=0.0f;} //Clear System
    //deltaT = mat4(1);
    globalError = 0;

    //cout<<"Before \n";
    //std::for_each(System.begin(), System.end(), [](float &a){cout<<a<<"\t";});
    //FindCorrespondences(sourceVerts, sourceNormals, destinationVerts, destinationNormals, correspondenceVerts,
    //  correspondenceNormals, deltaT, distThres, normalThres);
    FindCorrespondences2(sourceVerts, targetVerts, targetNormals, deltaT, distThres, corrImageCoords);
    //FindCorrespondences2(sourceDepth, targetDepth, deltaT, distThres, corrImageCoords);
    cout<<"\nGlobal correspondence error is : "<<globalError<<"\n";
    SDL_FillRect(surface, NULL, 0xFFFFFFFF);
    updateSurface();
    SDL_UpdateWindowSurface( window );
    SDL_Delay(1000);
    cout<<"Number of correspondence pairs : "<<numCorrPairs<<"\n";
    tracker.BuildLinearSystem(sourceVerts, targetVerts, targetNormals, corrImageCoords);

    getchar();//for pause

    tracker.PrintSystem();
    //Print said matrices

    deltaT = glm::make_mat4(tracker.getTransform().data());

    const auto temp_view = Matrix4x4f(glm::value_ptr(deltaT));
    //Print final transform
    cout << termcolor::green<< "Updated Eigen transform : \n"<<termcolor::reset;
    cout << temp_view << "\n";

    //if(globalError <= 0.0) {
    //  cout<<"\n\n"<<termcolor::bold<<termcolor::blue<<"Global error is zero. Stopping."<<termcolor::reset<<"\n";
    //  break;
    //}
  }
}

void CalculateNormals(const vector<vec3>& verts, vector<vec3>& normals) {
  for(int i=0;i<numRows;++i)  {
    for(int j=0;j<numCols; ++j) {
      const int index = i*numCols + j;
      normals[index] = vec3(MINF, MINF, MINF);
      if (j > 0 && j < numCols - 1 && i > 0 && i < numRows - 1) {
        const vec3 CC = verts[(i + 0)*numCols + (j + 0)];
        const vec3 PC = verts[(i + 1)*numCols + (j + 0)];
        const vec3 CP = verts[(i + 0)*numCols + (j + 1)];
        const vec3 MC = verts[(i - 1)*numCols + (j + 0)];
        const vec3 CM = verts[(i + 0)*numCols + (j - 1)];

        if (CC.x != MINF && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF) {
          const vec3 n = cross(PC - MC, CP - CM);
    			const float  l = length(n);
          if(l > 0.0f)  {
            normals[index] = vec3(n.x/l, n.y/l, n.z/l);
          }
        }
      }
    }
  }
}

void VertsFromDepth(const uint16_t* depthData, vector<vec3>& vertices) {
  for(int i=0;i<numRows;++i)  {
    for(int j=0;j<numCols; ++j) {
      const int index = i*numCols + j;
      float depth = depthData[index]/5000.0f;
      vec3 point = K_inv*vec3(j,i,1.0);
      point = point * depth;
      vertices[index] = vec3(point.x, point.y, point.z);
    }
  }
}

void CreatePyramid(const vector<vec3>& src, vector<vector<vec3>>& target, int level)
{
  if(level<1) return;
  int offset = pow(2,level);
  int w = numCols/offset;
  int h = numRows/offset;
  auto v = vector<vec3>(w*h);
  for(int i=0;i<w;++i)  {
    for(int j=0;j<h;++j)  {
      int index= j*w + i;
      int index2 = (j*offset)*w + (i*offset);
      v[index] = src[index2];
    }
  }
  CreatePyramid(src, target, level-1);
  target.push_back(v);
}

#endif //UTILS_HPP
