#ifndef UTILS_HPP
#define UTILS_HPP
#define GLM_ENABLE_EXPERIMENTAL

#include <vector>
#include <iostream>
#include <fstream>

#include "termcolor.hpp"
#include "DebugHelper.hpp"
#include "common.h"
#include "EigenUtil.h"
#include <SDL2/SDL.h>

using std::array;
using std::vector;
using std::cout;
using glm::vec3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;

uint16_t *img1 = nullptr;
uint16_t *img2 = nullptr;

static vector<vec3> sourceVerts;
static vector<vec3> destinationVerts;
static vector<vec3> sourceNormals(numCols*numRows);
static vector<vec3> destinationNormals(numCols*numRows);
static vector<vec3> correspondenceVerts(numCols*numRows);
static vector<vec3> correspondenceNormals(numCols*numRows);
static vector<float> correspondenceWeights(numRows*numCols, 0.0f);
static vector<bool> mask(numCols*numRows, false);

static mat4 deltaT = mat4(1);
static float globalError = 0.0f;
//Matrix4x4f computedTransform = deltaT;
static Matrix6x6f ATA;
static Vector6f ATb;

static SDL_Window *window = nullptr;
static SDL_Surface *surface = nullptr;

inline bool isValid(vec3& p) {
  return p.x != MINF;
}

inline ivec2 cam2screenPos(vec3 p) {
	float x = ((p.x * fx) / p.z) + cx;
	float y = ((p.y * fy) / p.z) + cy;
	return ivec2(x, y);
}

void VertsFromDepth(const uint16_t* depthData, vector<vec3>& vertices) {
  for(int i=0;i<numRows;++i)  {
    for(int j=0;j<numCols; ++j) {
      const int index = i*numCols + j;
      float depth = depthData[index]/5000.0f;
      float x = ((j - cx)*depth) / (float)fx;
    	float y = ((i - cy)*depth) / (float)fy;
      vertices.emplace_back(x,-y,-depth);

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

    for(int i=0;i<numRows;++i)  {
      for(int j=0;j<numCols; ++j) {
        const int index = i*numCols + j;
        correspondenceVerts[index] = vec3(MINF, MINF, MINF);
        correspondenceNormals[index] = vec3(MINF, MINF, MINF);

        vec3 p_in = sourceVerts[index];
        vec3 n_in = sourceNormals[index];

        vec3 transformedVert = deltaT*vec4(p_in, 1.0f);
        vec3 transformedNormal = deltaT*vec4(n_in, 1.0f);

        ivec2 screenPos = cam2screenPos(transformedVert);
        uint linearIdx = screenPos.x*numCols + screenPos.y;
        //if projected point within image bounds
        if(screenPos.x > 0 && screenPos.x < numCols && screenPos.y > 0 && screenPos.y < numRows) {
          
          vec3 p_dest = destinationVerts[linearIdx];
          vec3 n_dest = destinationNormals[linearIdx];

          if(isValid(p_dest) /*&& isValid(n_dest)*/)  {
            float d = glm::length(transformedVert - p_dest);
            float n = glm::dot(vec3(transformedVert - p_dest), n_dest);
            //cout<<linearIdx<<") n : "<<n<<", d : "<<d<<"\n";
            //float n = glm::dot(transformedNormal, n_dest);

            if(d <= distThres) {
              globalError += d;
              mask[index] = true;
              correspondenceWeights[index] = d;
              correspondenceVerts[index] = p_dest;
              correspondenceNormals[index] = n_dest;
            }
          }
        }
      }
    }
}

inline
float calculate_B(const vec3& n, const vec3& d, const vec3& s)  {
  return glm::dot(n,d) - glm::dot(n,s);
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

Matrix4x4f delinearizeTransform(const Vector6f& x) {
  Matrix4x4f res; res.setIdentity();

	//Rotation
	Matrix3x3f R = Eigen::AngleAxisf(x[0], Eigen::Vector3f::UnitZ()).toRotationMatrix()*
		Eigen::AngleAxisf(x[1], Eigen::Vector3f::UnitY()).toRotationMatrix()*
		Eigen::AngleAxisf(x[2], Eigen::Vector3f::UnitX()).toRotationMatrix();
	
  //Translation
	Eigen::Vector3f t = x.segment(3, 3);

	res.block(0, 0, 3, 3) = R;
  res.block(0, 3, 3, 1) = t;

	return res;
}

void updateSurface()  {
  for(int i=0;i<mask.size();++i)  {
    char* raw = surface->pixels;
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

void Align(uint iters)  {
  
  for(uint i=0;i<iters;++i) {

    std::cout<< "\n"<<termcolor::on_red<< "Iteration : "<<i << termcolor::reset << "\n";
    ClearVector(correspondenceVerts);
    ClearVector(correspondenceNormals);
    ClearVector(mask);
    //for(auto &i:System) {i=0.0f;} //Clear System
    ATA.setZero(); ATb.setZero();
    //deltaT = mat4(1);
    globalError = 0.0f;

    //cout<<"Before \n";
    //std::for_each(System.begin(), System.end(), [](float &a){cout<<a<<"\t";});
    FindCorrespondences(sourceVerts, sourceNormals, destinationVerts, destinationNormals, correspondenceVerts, 
      correspondenceNormals, deltaT, distThres, normalThres);
    cout<<"\nGlobal correspondence error is : "<<globalError<<"\n";
    uint numCorrPairs = 0;
    std::for_each(mask.begin(), mask.end(), [&](bool T){ 
      if(T) numCorrPairs++;
    });

    SDL_FillRect(surface, NULL, 0xFFFFFFFF);
    updateSurface();
    SDL_UpdateWindowSurface( window );
    SDL_Delay(1000);
    cout<<"Number of correspondence pairs : "<<numCorrPairs<<"\n";
    buildLinearSystem(sourceVerts, correspondenceVerts, correspondenceNormals, ATA, ATb);
    
    //cout<<"After \n";
    //std::for_each(System.begin(), System.end(), [](float &a){cout<<a<<"\t";});
    //build6x6Matrix(System, ATA, ATb);
    //Print said matrices
    std::cout << termcolor::green <<"\nFilled matrix system ATA | ATb : \n"<< termcolor::reset;
    for(int i=0;i<6;++i)  {
      for(int j=0;j<6;++j)  {
        std::cout<<ATA(i,j) <<" ";
      }
      std::cout<< "| "<<ATb(i)<<"\n";
    }

    //Now solve the system
    Eigen::JacobiSVD<Matrix6x6f> SVD(ATA, Eigen::ComputeFullU | Eigen::ComputeFullV);
  	Vector6f x = SVD.solve(ATb);

    cout << termcolor::green<< "Calculated solution vector : \n"<<termcolor::reset;
    cout << x <<"\n";

    Matrix4x4f newTransform = delinearizeTransform(x);
    glm::mat4 intermediateT = glm::make_mat4(newTransform.data());
    //Print final transform
    cout << termcolor::green<< "Calculated Eigen transform : \n"<<termcolor::reset;
    cout << newTransform << "\n";

    cout << termcolor::green<< "Copied GLM transform : \n"<<termcolor::reset;

    deltaT = intermediateT*deltaT;
    cout<<glm::to_string(deltaT)<<"\n";

    if(globalError == 0.0f) {
      cout<<"\n\n"<<termcolor::bold<<termcolor::blue<<"Global error is zero. Stopping."<<termcolor::reset<<"\n";
      break;
    }
  }
}

#endif //UTILS_HPP