#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>
#include <Eigen/StdVector>
#include "urbg2o.h"
#include "LocalBundleAdjustment.h"

namespace py = pybind11;

PYBIND11_PLUGIN(urbg2o)
{
    py::module m("urbg2o", "pybind11 opencv example plugin");

    m.def("poseOptimization", &poseOptimization, "pose-only bundle adjustment",
          py::arg("coords"), py::arg("pose"));
    m.def("LocalBundleAdjustment", &LocalBundleAdjustment, "local bundle adjustment",
          py::arg("keyframes"), py::arg("fixedKeyframes"), py::arg("worldMapPoints"), py::arg("pointsRelation"));

    return m.ptr();
}

