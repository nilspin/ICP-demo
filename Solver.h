#ifndef SOLVER_H
#define SOLVER_H

#include "common.h"
#include "EigenUtil.h"
#include "DebugHelper.hpp"

class Solver {
  public:
    uint numIters = 10; 
    
    void BuildLinearSystem(const vector<vec3>&, const vector<vec3>&, const vector<vec3>&, const vector<CoordPair>&);
    
    void PrintSystem();

    Matrix4x4f Solver::SolveJacobianSystem(const Matrix6x6f& JTJ, const Vector6f& JTr);
    
    Solver();

    Matrix4x4f getTransform() {return deltaT;};
  private:
    Vector6f X; //Will hold solution
    bool solution_exists = false;
    Matrix4x4f deltaT;  //intermediate estimated transform
    Vector6f Jacobian, JTr;  //for cost func [(T*src - dest)^2]
    double Residue;
    Matrix6x6f JTJ;
    float CalculateJacobianAndResidue(Vector6f& J, const vec3& srcVert, 
      const vec3& destVert, const vec3& destNormal);
    void ComputeJTJandJTr(const Vector6f& J, Vector6f& JTJ, Vector6f& JTr);
    Matrix4x4f DelinearizeTransform(const Vector6f& x);
};


#endif