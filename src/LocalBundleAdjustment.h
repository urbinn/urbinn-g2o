//
//  LocalBundleAdjustment.hpp
//  ORB_Display_Tool
//
//  Created by Daniello on 05/12/2017.
//  Copyright © 2017 DDdesigns. All rights reserved.
//


#ifndef ORBG2O_H
#define ORBG2O_H

#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/LU>
#include <Eigen/StdVector>

//int poseOptimization(Eigen::Ref<Eigen::MatrixXd> coords, Eigen::Ref<Eigen::MatrixXd> pose);

int LocalBundleAdjustment(Eigen::Ref<Eigen::MatrixXd> keyframes, Eigen::Ref<Eigen::MatrixXd> fixedKeyframes, Eigen::Ref<Eigen::MatrixXd> worldMapPoints, Eigen::Ref<Eigen::MatrixXd> pointsRelation ) ;

#endif

