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
vec3 Trans, Scale, Skew;
vec4 Perspective;
quat RotQuat;
mat3 Rot;

Solver tracker;
static vector<float> sourceDepth(numCols*numRows);
static vector<float> targetDepth(numCols*numRows);
static vector<vec3> sourceVerts(numCols*numRows);
static vector<vec3> targetVerts(numCols*numRows);
static vector<vec3> targetNormals(numCols*numRows);
static vector<CoordPair> corrImageCoords(numCols*numRows);
//static vector<float> correspondenceWeights(numRows*numCols, 0.0f);
static vector<float> errorMask(numCols*numRows);
//static vector<bool> mask(numCols*numRows, false);

static mat4 deltaT = mat4(1);
static double globalError = 0.0;

static SDL_Window *window = nullptr;
static SDL_Surface *surface = nullptr;

using namespace igl;

//declarations
template<typename T>
void CreatePyramid(const std::vector<T>& src, std::vector<vector<T>>& target, int level);

void SetupCameraIntrinsic() {
  K = glm::make_mat3(intrinsics);
  K = transpose(K);
  K_inv = inverse(K);
}


void updateSurface()  {
  SDL_FillRect(surface, NULL, 0xFFFFFFFF);
  char* raw = reinterpret_cast<char*>(surface->pixels);
  auto minIt = std::min_element(errorMask.begin(), errorMask.end());
  auto maxIt = std::max_element(errorMask.begin(), errorMask.end());
  float min = *minIt;
  float max = *maxIt;
  float val;
  float r,g,b;
  for(int i=0; i< (numRows*numCols); ++i)  {
    val = (errorMask[i] - min)/(max-min);
    char col = val*256;
    //uint w = sizeof()
    //uint16_t in = img1[i];
    //char d = in;
    //float d = correspondenceWeights[i]*0.05;
    //if(mask[i]) {
      //colormap(COLOR_MAP_TYPE_VIRIDIS, val, r, g, b);
      //std::cout<<"r"<<r;
      //std::cout<<"g"<<g;
      //std::cout<<"b"<<b<<"\n";
      raw[4*i] = col; //b
      raw[(4*i)+1] = col; //g
      raw[(4*i)+2] = col; //r
      raw[(4*i)+3] = 255; //a
    //}
  }
  SDL_UpdateWindowSurface( window );
  SDL_Delay(1000);
}

inline ivec2 cam2screenPos(vec3 p) {
  vec3 sp = K*p;
  //ivec2 spos = ivec2(sp.x, sp.y);
  ivec2 spos = ivec2(sp.x/sp.z + 0.5, sp.y/sp.z + 0.5);
  //ivec2 spos = ivec2(sp.x/sp.z, sp.y/sp.z);
  //std::cout<< glm::to_string(spos)  <<"\n";
  return spos;
}

void FindCorrespondences2(const vector<vec3>& src, const vector<vec3>& targ, const vector<vec3>& targNormals, const mat3& Rot, const vec3& Trans, float distThres, vector<CoordPair>& correspondencePairs, int level)  {

  numCorrPairs = 0;

  int offset = pow(2,level);
  int w = numCols/offset;
  int h = numRows/offset;
  std::cout<<"numCols:"<<numCols<<" ,w:"<<w<<"\n";
  std::cout<<"numRows:"<<numRows<<" ,h:"<<h<<"\n";
  std::cout<<"offset:"<<offset<<"\n";
  std::cout<<"level:"<<level<<"\n";

  for(uint v_s = 0; v_s < h; ++v_s)  {
    for(uint u_s = 0; u_s < w; ++u_s)  {
      uint index = v_s*w + u_s;
      vec3 pSrc = src[index];
      vec3 transPSrc = (Rot * pSrc) + Trans;//transform
      vec3 projected = K * (transPSrc);
      int u_t = (int)(((projected.x / projected.z) + 0.5)/offset);//non-homogenize
      int v_t = (int)(((projected.y / projected.z) + 0.5)/offset);
      if (u_t >= 0 && u_t < w && v_t >= 0 && v_t < h) {
        uint targetIndex = v_t*w + u_t;
        vec3 pTar = targ[targetIndex];
        vec3 nTar = targNormals[targetIndex];
        vec3 diff = transPSrc - pTar;
        double d = glm::dot(diff, nTar);
        if ((d < distThres)) {
          numCorrPairs++;
          //std::cout<<numCorrPairs<<" - src: "<<glm::to_string(ivec2(u_s, v_s))<<" , target: "<<glm::to_string(ivec2(u_t, v_t))<<"\n";
          errorMask[index*offset] = d;
          //errorMask[v_s*numCols + u_s] = d;
          //mask[index*offset] = true;
          correspondencePairs.push_back(std::make_tuple(ivec2(u_s, v_s), ivec2(u_t, v_t), d));
        }
      }
    }
  }
}

void Align(uint iters)  {

  auto srcVerts_pyramid = vector<vector<vec3>>();
  auto targetVerts_pyramid = vector<vector<vec3>>();
  auto targetNormals_pyramid = vector<vector<vec3>>();
  auto corrImageCoords_pyramid = vector<vector<CoordPair>>();
  CreatePyramid(sourceVerts, srcVerts_pyramid, pyramid_size);
  CreatePyramid(targetVerts, targetVerts_pyramid, pyramid_size);
  CreatePyramid(targetNormals, targetNormals_pyramid, pyramid_size);
  //CreatePyramid(corrImageCoords, corrImageCoords_pyramid, pyramid_size);
  corrImageCoords_pyramid.push_back(vector<CoordPair>());
  corrImageCoords_pyramid.push_back(vector<CoordPair>());
  corrImageCoords_pyramid.push_back(vector<CoordPair>());

  std::cout<<"Pyramid size "<<srcVerts_pyramid.size()<<"\n";
  int pyrSize = srcVerts_pyramid.size();
  for(int i=0;i<pyrSize;++i) {
    std::cout<<"level"<<i<<" size "<<srcVerts_pyramid[i].size()<<"\n";
  }

  for(uint iter=0; iter <= maxIters; ++iter) {

    uint lvl = 0;

    std::cout<< "\n"<<termcolor::on_red<< "Iteration : "<< iter << termcolor::reset << "\n";
    //ClearVector(correspondenceVerts);
    //ClearVector(correspondenceNormals);
    //ClearVector(mask);
    ClearVector(errorMask);
    //ClearVector(corrImageCoords_pyramid[lvl]);
    corrImageCoords_pyramid[lvl].clear();
    globalError = 0;

    glm::decompose(deltaT, Scale, RotQuat, Trans, Skew, Perspective);
    RotQuat = glm::conjugate(RotQuat);
    Rot = glm::toMat3(RotQuat);

    FindCorrespondences2(srcVerts_pyramid[lvl], targetVerts_pyramid[lvl], targetNormals_pyramid[lvl], Rot, Trans, distThres, corrImageCoords_pyramid[lvl], lvl);

    updateSurface();
    cout<<"Number of correspondence pairs : "<<numCorrPairs<<"\n";
    tracker.BuildLinearSystem(srcVerts_pyramid[lvl], targetVerts_pyramid[lvl], targetNormals_pyramid[lvl], corrImageCoords_pyramid[lvl], lvl);

    //getchar();//for pause

    //tracker.PrintSystem();
    //Print said matrices

    globalError = tracker.getError();
    cout<<"\nGlobal correspondence error is : "<<globalError<<"\n";
    deltaT = glm::make_mat4(tracker.getTransform().data());
    deltaT = glm::transpose(deltaT);

    const auto temp_view = Matrix4x4f(glm::value_ptr(deltaT));
    //Print final transform
    //cout << termcolor::green<< "Updated Eigen transform : \n"<<termcolor::reset;
    //cout << temp_view << "\n";

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

template<typename T>
void CreatePyramid(const vector<T>& src, vector<vector<T>>& target, int level)
{
  if(level<0) return;
  int offset = pow(2,level);
  int w = numCols/offset;
  int h = numRows/offset;
  auto v = vector<T>(w*h);
  for(int j=0;j<h;++j)  {
    for(int i=0;i<w;++i)  {
      int index= j*w + i;
      v[index] = src[index*offset];
    }
  }
  CreatePyramid(src, target, level-1);
  target.push_back(v);
}

#endif //UTILS_HPP
