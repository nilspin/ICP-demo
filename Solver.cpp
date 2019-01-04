#include "Solver.h"
#include "termcolor.hpp"

inline bool isValid(vec3& p) {
  return p.x != 0.0f;
}

inline
float calculate_B(const vec3& n, const vec3& d, const vec3& s)  {
  return glm::dot(n,d) - glm::dot(n,s);
}

void Solver::PrintSystem()  {
  std::cout << termcolor::green <<"\nFilled matrix system JTJ | JTr : \n"<< termcolor::reset;
  for(int i=0;i<6;++i)  {
    for(int j=0;j<6;++j)  {
      std::cout<<JTJ(i,j) <<" ";
    }
    std::cout<< "| "<<JTr(i)<<"\n";
  }

  cout << termcolor::green<< "Calculated solution vector : \n"<<termcolor::reset;
  cout << X <<"\n";

}
float Solver::CalculateJacobianAndResidue(Vector6f& J, const vec3& d_n, const vec3& d, const vec3& s)  {
  vec3 T = cross(s, d_n);
  J[0] = T.x;
  J[1] = T.y;
  J[2] = T.z;
  J[3] = -d_n.x;
  J[4] = -d_n.y;
  J[5] = -d_n.z;
  return calculate_B(d_n, d, s);
}

void Solver::BuildLinearSystem(const vector<vec3>& sourceVerts, const vector<vec3>& destVerts, const vector<vec3>& destNormals, const vector<CoordPair>& corrImageCoords) {

  auto& J = Jacobian;
  J.setZero();
  JTJ.setZero();
  JTr.setZero();
  uint linIdx=0;
  for(uint i=0;i<corrImageCoords.size(); ++i)  {
    ivec2 srcCoord = std::get<0>(corrImageCoords[i]);
    ivec2 targetCoord = std::get<1>(corrImageCoords[i]);
    uint srcIndex = srcCoord.y*numCols + srcCoord.x;
    uint targetIndex = targetCoord.y*numCols + targetCoord.x;
    vec3 s = sourceVerts[srcIndex];
    vec3 d = destVerts[targetIndex];
    vec3 n = destNormals[targetIndex];

    if(isValid(n)) {
      float residue = CalculateJacobianAndResidue(J, n, d, s);
      
      linIdx=0;
        
      //if(i==32304)  {
      //  cout<<"n)"<<glm::to_string(n)<<", d)"<<glm::to_string(d)<<", s)"<<glm::to_string(s)<<"\n";
      //}
      //We now have enough information to build Ax=b system. Let's calculate JTJ and JTr
      for(uint j=0;j<6;++j)  {
        for(uint k=0;k<6;++k)  {
          //36 elements for matrix of 6x6 JTJ
          JTJ(j,k) += J[j]*J[k];
        }
        JTr(j) += J[j]*residue; //For 6x1 JTr
      }
    }
  }

  //Our system is built. Solve it
  deltaT = SolveJacobianSystem(JTJ, JTr);
}

Matrix4x4f Solver::SolveJacobianSystem(const Matrix6x6f& JTJ, const Vector6f& JTr)  {
  X.setZero();
  //first check if solution exists
  float det = JTJ.determinant();
  if (std::abs(det) < 1e-6 || std::isnan(det) || std::isinf(det)) {
      solution_exists = false;
  }
  if (solution_exists) {
    // Robust Cholesky decomposition of a matrix with pivoting.
    X = JTJ.ldlt().solve(-JTr);
  }
  Matrix4x4f tempTransform = DelinearizeTransform(X);
  return tempTransform;
}

Matrix4x4f Solver::DelinearizeTransform(const Vector6f& x) {
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

Solver::Solver() {
  Jacobian.setZero();
  JTJ.setZero();
  JTr.setZero();  
  deltaT.setZero();
}