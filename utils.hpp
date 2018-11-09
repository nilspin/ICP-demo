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

using std::array;
using std::vector;
using std::cout;
using glm::vec3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;
using linearSystem = std::array<float,27> ;

static vector<vec3> sourceVerts;
static vector<vec3> destinationVerts;
static vector<vec3> sourceNormals(numCols*numRows);
static vector<vec3> destinationNormals(numCols*numRows);
static vector<vec3> correspondenceVerts(numCols*numRows);
static vector<vec3> correspondenceNormals(numCols*numRows);
static mat4 deltaT = mat4(1);
static float globalError = 0.0f;
static linearSystem System;
//Matrix4x4f computedTransform = deltaT;
static Matrix6x6f ATA;
static Vector6f ATb;


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

          if(isValid(p_dest) && isValid(n_dest))  {
            float d = glm::length(transformedVert - p_dest);
            float n = glm::dot(transformedNormal, n_dest);

            if(d <= distThres) {
              globalError += d;
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
 linearSystem& system)  {

   float A[6] = {0.0f};
   uint k=0;
   for(uint i=0;i<sourceVerts.size(); ++i)  {
     vec3 s = sourceVerts[i];
     vec3 d = destinationVerts[i];
     vec3 n = destinationNormals[i];

     if(isValid(n)) {
        k=0;
        float b = calculate_B(n, d, s);
        vec3 T = glm::cross(s, n);
        A[0] = T.x;
        A[1] = T.y;
        A[2] = T.z;
        A[3] = n.x;
        A[4] = n.y;
        A[5] = n.z;
        if(i==32304)  {
          cout<<"n)"<<glm::to_string(n)<<", d)"<<glm::to_string(d)<<", s)"<<glm::to_string(s)<<"\n";
        }
        //We now have enough information to build Ax=b system. Let's calculate ATA and ATb
        for(uint j=0;j<6;++j)  {
          for(uint k=j;k<6;++k)  {
            //21 elements for upper triangular matrix of 6x6 ATA
            system[k++] += A[j]*A[k];
          }
          system[21+j] += A[j]*b; //For ATb
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

void build6x6Matrix(const linearSystem& sys, Matrix6x6f& AtA, Vector6f& Atb)  {
  uint k=0;
  //Fill upper triangluar matrix
  for(uint i=0;i<6;++i) {
    for(uint j=i;j<6;++j) {
      AtA(i,j) = sys[k++];
    }
  }

  //Copy to lower triangular matrix
  for (int i = 0; i < 6; ++i) {
		for (int j = i; j < 6; ++j) {
			AtA(j, i) = AtA(i, j);
		}
	}

  //fill Atb
  for(int i=0;i<6;++i)  {
    Atb(i) = sys[21 + i];
  }
}

void Align(uint iters)  {
  
  for(uint i=0;i<iters;++i) {
    std::cout<< "\n"<<termcolor::on_red<< "Iteration : "<<i << termcolor::reset << "\n";
    ClearVector(correspondenceVerts);
    ClearVector(correspondenceNormals);
    for(auto &i:System) {i=0.0f;} //Clear System
    ATA.setZero(); ATb.setZero();
    //deltaT = mat4(1);
    globalError = 0.0f;

    cout<<"Before \n";
    std::for_each(System.begin(), System.end(), [](float &a){cout<<a<<"\t";});
    FindCorrespondences(sourceVerts, sourceNormals, destinationVerts, destinationNormals, correspondenceVerts, 
      correspondenceNormals, deltaT, distThres, normalThres);
    cout<<"Global correspondence error is : "<<globalError<<"\n";

    buildLinearSystem(sourceVerts, destinationVerts, destinationNormals, System);
    
    cout<<"After \n";
    std::for_each(System.begin(), System.end(), [](float &a){cout<<a<<"\t";});
    build6x6Matrix(System, ATA, ATb);
    //Print said matrices
    std::cout << termcolor::green <<"Filled matrix system ATA | ATb : \n"<< termcolor::reset;
    for(int i=0;i<6;++i)  {
      for(int j=0;j<6;++j)  {
        std::cout<<ATA(i,j) <<" ";
      }
      std::cout<< "| "<<ATb(i)<<"\n";
    }

    //Now solve the system
    Eigen::JacobiSVD<Matrix6x6f> SVD(ATA, Eigen::ComputeFullU | Eigen::ComputeFullV);
  	Vector6f x = SVD.solve(ATb);
    
    Matrix4x4f newTransform = delinearizeTransform(x);
    //Print final transform
    cout << termcolor::green<< "Calculated Eigen transform : \n"<<termcolor::reset;
    cout << newTransform << "\n";

    cout << termcolor::green<< "Copied GLM transform : \n"<<termcolor::reset;
    deltaT = glm::make_mat4(newTransform.data());
  }
}

#endif //UTILS_HPP