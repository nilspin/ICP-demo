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
#include "Solver.h"
#include <SDL2/SDL.h>

uint numCorrPairs = 0;
uint16_t *img1 = nullptr;
uint16_t *img2 = nullptr;
mat3 K, K_inv;  //Camera intrinsic matrix, its inverse
mat3 Rot;
vec3 Trans;
Solver tracker;
static vector<float> sourceDepth(640*480);
static vector<float> targetDepth(640*480);
static vector<vec3> sourceVerts(640*480);
static vector<vec3> targetVerts(640*480);
static vector<vec3> targetNormals(640*480);
static vector<CoordPair> corrImageCoords;
//static vector<float> correspondenceWeights(numRows*numCols, 0.0f);
static vector<bool> mask(numCols*numRows, false);

static mat4 deltaT = mat4(1);
static double globalError = 0.0;

static SDL_Window *window = nullptr;
static SDL_Surface *surface = nullptr;


void SetupCameraIntrinsic() {
  K = glm::make_mat3(intrinsics);
  K = transpose(K);
  K_inv = inverse(K);
}


void updateSurface()  {
  for(int i=0;i<mask.size();++i)  {
    char* raw = reinterpret_cast<char*>(surface->pixels);
    //uint w = sizeof()
    //uint16_t in = img1[i];
    //char d = in;
    //float d = correspondenceWeights[i]*0.05;
    if(mask[i]) {
      raw[4*i] = 0; //b
      raw[(4*i)+1] = 0; //g
      raw[(4*i)+2] = 0; //r
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

void FindCorrespondences2(const vector<float>& s_depth, const vector<float>& t_depth, const mat4& deltaT, float distThres, vector<CoordPair>& corrImageCoords)  {
  vec3 Trans, Scale, Skew;
  vec4 Perspective;
  quat RotQuat;
  mat3 Rot;
  glm::decompose(deltaT, Scale, RotQuat, Trans, Skew, Perspective);
  Rot = glm::toMat3(RotQuat);
  mat3 KRK_inv = K * Rot * K_inv;
  vec3 Kt = K * Trans;
  numCorrPairs = 0;

  for(uint v_s = 0; v_s < numRows; ++v_s)  {
    for(uint u_s = 0; u_s < numCols; ++u_s)  {
      uint index = v_s*numCols + u_s;
      float d_s = s_depth[index];
      vec3 uv_in_s = d_s * KRK_inv * vec3(u_s, v_s, 1.0) + Kt;
      float transformed_d_s = uv_in_s.z;
      int u_t = (int)(uv_in_s.x / transformed_d_s + 0.5);
      int v_t = (int)(uv_in_s.y / transformed_d_s + 0.5);
      if (u_t >= 0 && u_t < numCols && v_t >= 0 && v_t < numRows) {
        uint targetIndex = v_t*numCols + u_t;
        float d_t = targetDepth[targetIndex];
        double d = std::abs(transformed_d_s - d_t);
        if (!std::isnan(d_t) && (d < distThres)) {
          numCorrPairs++;
          //std::cout<<numCorrPairs<<" - src: "<<glm::to_string(ivec2(u_s, v_s))<<" , target: "<<glm::to_string(ivec2(u_t, v_t))<<"\n";
          globalError += d;
          mask[index] = true;
          corrImageCoords.push_back(std::make_tuple(ivec2(u_s, v_s), ivec2(u_t, v_t))); 
        }
      }
    }
  }
}

void Align(uint iters)  {
  
  for(uint i=0;i<iters;++i) {

    std::cout<< "\n"<<termcolor::on_red<< "Iteration : "<<i << termcolor::reset << "\n";
    //ClearVector(correspondenceVerts);
    //ClearVector(correspondenceNormals);
    ClearVector(mask);
    corrImageCoords.clear();
    //for(auto &i:System) {i=0.0f;} //Clear System
    //deltaT = mat4(1);
    globalError = 0;

    //cout<<"Before \n";
    //std::for_each(System.begin(), System.end(), [](float &a){cout<<a<<"\t";});
    //FindCorrespondences(sourceVerts, sourceNormals, destinationVerts, destinationNormals, correspondenceVerts, 
    //  correspondenceNormals, deltaT, distThres, normalThres);
    FindCorrespondences2(sourceDepth, targetDepth, deltaT, distThres, corrImageCoords);
    cout<<"\nGlobal correspondence error is : "<<globalError<<"\n";
    cout<<"\nCorrImageCoordSize is : "<<corrImageCoords.size()<<"\n";
    SDL_FillRect(surface, NULL, 0xFFFFFFFF);
    updateSurface();
    SDL_UpdateWindowSurface( window );
    SDL_Delay(1000);
    cout<<"Number of correspondence pairs : "<<numCorrPairs<<"\n";
    /*
    buildLinearSystem(sourceVerts, correspondenceVerts, correspondenceNormals, ATA, ATb);
    */
    tracker.BuildLinearSystem(sourceVerts, targetVerts, targetNormals, corrImageCoords);

    std::system("pause");
    tracker.PrintSystem();
    //Print said matrices
    
    Matrix4x4f newTransform = tracker.getTransform();
    /*
    newTransform = delinearizeTransform(x);
    */
    glm::mat4 intermediateT = glm::make_mat4(newTransform.data());
    //Print final transform
    cout << termcolor::green<< "Calculated Eigen transform : \n"<<termcolor::reset;
    cout << newTransform << "\n";

    //cout << termcolor::green<< "Copied GLM transform : \n"<<termcolor::reset;

    deltaT = intermediateT*deltaT;
    //cout<<glm::to_string(deltaT)<<"\n";

    if(globalError == 0.0) {
      cout<<"\n\n"<<termcolor::bold<<termcolor::blue<<"Global error is zero. Stopping."<<termcolor::reset<<"\n";
      break;
    }
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
      vertices.emplace_back(point.x, -point.y, -point.z);
    }
  }
}

/*

static vector<vec3> sourceVerts;
static vector<vec3> destinationVerts;
static vector<vec3> sourceNormals(numCols*numRows);
static vector<vec3> destinationNormals(numCols*numRows);
static vector<vec3> correspondenceVerts(numCols*numRows);
static vector<vec3> correspondenceNormals(numCols*numRows);

void VertsFromDepth(const uint16_t* depthData, vector<vec3>& vertices) {
  for(int i=0;i<numRows;++i)  {
    for(int j=0;j<numCols; ++j) {
      const int index = i*numCols + j;
      float depth = depthData[index]/5000.0f;
      vec3 point = K_inv*vec3(j,i,1.0);
      point = point * depth;
      vertices.emplace_back(point.x, -point.y, -point.z);
    }
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

void FindCorrespondences(
  const vector<vec3>& sourceVerts, const vector<vec3>& sourceNormals,
  const vector<vec3>& destinationVerts, const vector<vec3>& destinationNormals,
  vector<vec3>& correspondenceVerts, vector<vec3>& correspondenceNormals,
  const mat4& deltaT, float distThres, float normalThres)  {

    fill(correspondenceVerts.begin(), correspondenceVerts.end(), vec3(MINF, MINF, MINF));
    //fill(correspondenceNormals.begin(), correspondenceNormals.end(), vec3(MINF, MINF, MINF));

    vec3 Trans, Scale, Skew;
    vec4 Perspective;
    quat RotQuat;
    mat3 Rot;
    glm::decompose(deltaT, Scale, RotQuat, Trans, Skew, Perspective);
    Rot = glm::toMat3(RotQuat);
    mat3 KRK_inv = K * Rot * K_inv;
    vec3 Kt = K * Trans;
    std::cout<<"deltaT : "<<glm::to_string(deltaT)<<"\n";
    std::cout<<"RotQuat : "<<glm::to_string(RotQuat)<<"\n";
    std::cout<<"Rotation : "<<glm::to_string(Rot)<<"\n";
    std::cout<<"Translation : "<<glm::to_string(Trans)<<"\n";
    
    for(int i=0;i<numRows;++i)  {
      for(int j=0;j<numCols; ++j) {
        const int index = i*numCols + j;
        
        vec3 p_in = sourceVerts[index];
        //vec3 n_in = sourceNormals[index];

        vec3 transformedSource = deltaT*vec4(p_in, 1.0f);
        //vec3 transformedNormal = deltaT*vec4(n_in, 1.0f);
        
        //std::cout<<index<<" : ";
        ivec2 screenPos = cam2screenPos(transformedSource);
        uint targetIndex = screenPos.y*numCols + screenPos.x;
        //if projected point within image bounds
        if(screenPos.x > 0 && screenPos.x < numCols && screenPos.y > 0 && screenPos.y < numRows) {
          
          vec3 p_dest = destinationVerts[targetIndex];
          //vec3 n_dest = destinationNormals[targetIndex];
          float d = std::abs(transformedSource - p_dest);

          if(isValid(p_dest) && d<distThres)  {

            numCorrPairs++;            
            //std::cout<<numCorrPairs<<" - "<<glm::to_string(p_dest)<<" "<<glm::to_string(p_in)<<"\n";
            std::cout<<numCorrPairs<<" - src: "<<glm::to_string(ivec2(j,i))<<" , target: "<<glm::to_string(screenPos)<<"\n";
            //float n = glm::dot(vec3(transformedSource - p_dest), n_dest);
            //n = std::abs(n*n);
            globalError += d;
            mask[index] = true;
            //correspondenceWeights[index] = d;
            correspondenceVerts[index] = p_dest;
            //correspondenceNormals[index] = n_dest;
            //cout<<linearIdx<<") n : "<<n<<", d : "<<d<<"\n";
          }
        }
      }
    }
}



void buildLinearSystem(const vector<vec3>& sourceVerts, const vector<vec3>& destVerts, const vector<vec3>& destNormals,
 Matrix6x6f& AtA, Vector6f& Atb)  {

   float A[6] = {0.0f};
   uint linIdx=0;
   for(uint i=0;i<sourceVerts.size(); ++i)  {
     vec3 s = sourceVerts[i];
     vec3 d = destVerts[i];
     vec3 n = destNormals[i];

     if(isValid(n)) {
        linIdx=0;
        float b = calculate_B(n, d, s);
        vec3 T = glm::cross(s, n);
        A[0] = T.x;
        A[1] = T.y;
        A[2] = T.z;
        A[3] = -n.x;
        A[4] = -n.y;
        A[5] = -n.z;
        //if(i==32304)  {
        //  cout<<"n)"<<glm::to_string(n)<<", d)"<<glm::to_string(d)<<", s)"<<glm::to_string(s)<<"\n";
        //}
        //We now have enough information to build Ax=b system. Let's calculate ATA and ATb
        for(uint j=0;j<6;++j)  {
          for(uint k=0;k<6;++k)  {
            //36 elements for matrix of 6x6 ATA
            AtA(j,k) += A[j]*A[k];
          }
          Atb(j) += A[j]*b; //For 6x1 ATb
        }
    }
  }
}


*/
#endif //UTILS_HPP