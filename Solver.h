#ifndef SOLVER_H
#define SOLVER_H

#include "common.h"
#include "EigenUtil.h"
#include "DebugHelper.hpp"

using namespace Eigen;

class Solver {
  public:
    uint numIters = 10;

    void BuildLinearSystem(const vector<vec3>&, const vector<vec3>&, const vector<vec3>&, const vector<CoordPair>&, int);

    void PrintSystem();

    void SolveJacobianSystem(const Matrix6x6f& JTJ, const Vector6f& JTr);

    Solver();

    Matrix4x4f getTransform() {return SE3Exp(estimate);};
    double getError() {return TotalError;};
  private:
    Vector6f update, estimate; //Will hold solution
    bool solution_exists = false;
    //Matrix4x4f deltaT;  //intermediate estimated transform
    Vector6f JTr;  //for cost func [(T*src - dest)^2]
    MatrixXf Jac;
    VectorXf residual;
    double TotalError;
    int numCorrPairs = 0;
    Matrix6x6f JTJ, JTJinv;
    void CalculateJacobians(MatrixXf& J, const vec3& destVert,
        const vec3& destNormal, int index);
    void ComputeJTJandJTr(const Vector6f& J, Vector6f& JTJ, Vector6f& JTr);
    Matrix4x4f DelinearizeTransform(const Vector6f& x);
};


#endif
