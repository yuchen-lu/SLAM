//
// Created by yuchen on 09/01/18.
//





#include <Eigen/StdVector>
#include <Eigen/Core>

#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <memory>
#include <vector>
#include <stdlib.h>

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "common/BundleParams.h"
#include "common/BALProblem.h"
#include "g2o_bal_class.h"


// solve g2o



typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3> > BalBlockSolver;

void SetSolverOptionsFromFlags(BALProblem* bal_problem, const BundleParams& params,
 g2o::SparseOptimizer* optimizer)
{
    BalBlockSolver* solver_ptr;
    g2o::LinearSolver<BalBlockSolver::PoseMatrixType >* linearSolver = 0;

    // use dense calculate method
    if (params.linear_solver == "dense_schur"){
        linearSolver = new g2o::LinearSolverDense<BalBlockSolver::PoseMatrixType >();
    }
}
