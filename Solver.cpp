#include "Solver.h"
#include "termcolor.hpp"
#include <string>

inline
float calculate_B(const vec3& n, const vec3& d, const vec3& s)  {
  glm::vec3 p = vec3(d - s);
  return glm::dot(p,n);
}

inline
void PrintMatrixDims(const MatrixXf& M, const std::string& s) {
  std::cout<<"Size of "<<s<<" matrix is : "<<M.rows()<<"x"<<M.cols()<<"\n";
}

inline
void PrintMatrix(const MatrixXf& M, const std::string& s) {
  std::cout<<"Matrix "<<s<<" : \n"<<M<<"\n";
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
  cout << estimate <<"\n";

}

void Solver::CalculateJacobians(MatrixXf& JacMat, const vec3& d, const vec3& n, int index)  {
  vec3 T = cross(d, n);
  // Calculate Jacobian for this correspondence. Probably most important piece of code
  // in entire project
  JacMat.row(index) << n.x, n.y, n.z, T.x, T.y, T.z ;
}

void Solver::BuildLinearSystem(const vector<vec3>& sourceVerts, const vector<vec3>& destVerts, const vector<vec3>& destNormals, const vector<CoordPair>& corrImageCoords, int level) {

  int offset = pow(2,level);
  int w = numCols/offset;
  int h = numRows/offset;

  numCorrPairs = corrImageCoords.size();
  Jac = MatrixXf(numCorrPairs,6);
  residual = VectorXf(numCorrPairs);
  //PrintMatrixDims(Jac, std::string("Jac"));
  //PrintMatrixDims(residual, std::string("residual"));
  //PrintMatrixDims(JTJ, std::string("JTJ"));
  //PrintMatrixDims(JTr, std::string("JTr"));
  auto& J = Jac;  //Jacobian;
  J.setZero();
  JTJ.setZero();
  JTr.setZero();
  residual.setZero();
  uint idx = 0;

  for(auto const& iter : corrImageCoords)  {
    ivec2 srcCoord = std::get<0>(iter);
    ivec2 targCoord = std::get<1>(iter);
    float r = std::get<2>(iter);
    //if(std::abs(r) == 0)  {
    //  continue;
    //}
    //std::cout<<"bla "<<r<<"\n";
    residual.row(idx) << r;  //std::vector to eigen mat
    int srcIndex = srcCoord.y*w + srcCoord.x;
    int targetIndex = targCoord.y*w + targCoord.x;
    if(srcIndex >= 0 && srcIndex < w*h && targetIndex >= 0 && targetIndex < w*h)
    {
      vec3 s = sourceVerts[srcIndex];
      vec3 d = destVerts[targetIndex];
      vec3 n = destNormals[targetIndex];
      CalculateJacobians(J, d, n, idx);
      idx++;
    }
  }
  //We have jacobian and residual. Make a linear system.
  JTJ = Jac.transpose() * Jac;  //should be 6x6
  JTr = Jac.transpose() * residual;
  JTJinv = JTJ.inverse();
  update = -(JTJinv * JTr);
  estimate = SE3Log(SE3Exp(update) * SE3Exp(estimate) );

  //SolveJacobianSystem(JTJ, JTr);

  TotalError = residual.transpose() * residual;

  //Print it
  //PrintMatrix(residual, "residual");
  //PrintMatrix(Jac, "Jac");
  //PrintMatrix(JTJ, "JTJ");
  //PrintMatrix(JTr, "JTr");
  //PrintMatrix(update, "update");
  //Our system is built. Solve it
}

void Solver::SolveJacobianSystem(const Matrix6x6f& JTJ, const Vector6f& JTr)  {
  update.setZero();
  //first check if solution exists
  float det = JTJ.determinant();
  if (std::abs(det) < 1e-6 || std::isnan(det) || std::isinf(det)) {
      solution_exists = false;
  }
  else {
    solution_exists = true;
    // Robust Cholesky decomposition of a matrix with pivoting.
    update = JTJ.ldlt().solve(-JTr);
  }
  estimate = SE3Log(SE3Exp(update) * SE3Exp(estimate) );
}
//
//Matrix4x4f Solver::DelinearizeTransform(const Vector6f& x) {
//  Matrix4x4f res; res.setIdentity();
//
//	//Rotation
//	Matrix3x3f R = Eigen::AngleAxisf(x[0], Eigen::Vector3f::UnitZ()).toRotationMatrix()*
//		Eigen::AngleAxisf(x[1], Eigen::Vector3f::UnitY()).toRotationMatrix()*
//		Eigen::AngleAxisf(x[2], Eigen::Vector3f::UnitX()).toRotationMatrix();
//
//  //Translation
//	Eigen::Vector3f t = x.segment(3, 3);
//
//	res.block(0, 0, 3, 3) = R;
//  res.block(0, 3, 3, 1) = t;
//
//	return res;
//}

Solver::Solver() {
  JTJ.setZero();
  JTr.setZero();
  //deltaT.setZero();
}
