cmake_minimum_required(VERSION 3.9)
project(ch10)

set(CMAKE_CXX_STANDARD 11)

add_executable(ch10
        g2o_custombundle/ceres/autodiff.h
        g2o_custombundle/ceres/eigen.h
        g2o_custombundle/ceres/fixed_array.h
        g2o_custombundle/ceres/fpclassify.h
        g2o_custombundle/ceres/jet.h
        g2o_custombundle/ceres/macros.h
        g2o_custombundle/ceres/manual_constructor.h
        g2o_custombundle/ceres/variadic_evaluate.h
        g2o_custombundle/common/flags/command_args.cpp
        g2o_custombundle/common/flags/command_args.h
        g2o_custombundle/common/tools/random.h
        g2o_custombundle/common/tools/rotation.h
        g2o_custombundle/common/BALProblem.cpp
        g2o_custombundle/common/BALProblem.h
        g2o_custombundle/common/BundleParams.h
        g2o_custombundle/common/projection.h
        g2o_custombundle/g2o_bal_class.h
        g2o_custombundle/g2o_bundle.cpp)
