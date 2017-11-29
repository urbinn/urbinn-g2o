#ifndef ORBG2O_H
#define ORBG2O_H

#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/LU>
#include <Eigen/StdVector>

int poseOptimization(Eigen::Ref<Eigen::MatrixXd> coords, Eigen::Ref<Eigen::MatrixXd> pose);

#endif
