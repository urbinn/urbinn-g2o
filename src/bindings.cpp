#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>
#include <Eigen/StdVector>
#include "urbg2o.h"

namespace py = pybind11;

PYBIND11_PLUGIN(urbg2o)
{
    py::module m("urbg2o", "pybind11 opencv example plugin");

    m.def("poseOptimization", &poseOptimization, "pose-only bundle adjustment",
	py::arg("coords").noconvert(), py::arg("pose"));
    
    return m.ptr();
}

