#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>

#include <unordered_set>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include <boost/python.hpp>

typedef boost::python::extract<int> extract_int;

class BundleAdjustment {
public:
    BundleAdjustment(boost::python::list a, boost::python::list b, int a_len, int b_len) {
        optimizer = std::make_unique<g2o::SparseOptimizer>();
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        std::vector<std::shared_ptr<g2o::VertexSBAPointXYZ>> mappoints = get_mappoints(b, b_len);
        std::vector<std::shared_ptr<g2o::SE3Quat>> poses = get_poses();
        std::vector<std::shared_ptr<g2o::EdgeProjectXYZ2UV>> edges = get_edges(poses, mappoints);
    }
private:
    std::vector<std::shared_ptr<g2o::EdgeProjectXYZ2UV>> get_edges(std::vector<std::shared_ptr<g2o::SE3Quat>> poses, std::vector<std::shared_ptr<g2o::VertexSBAPointXYZ>> mappoints) {
        
    }

    std::vector<std::shared_ptr<g2o::SE3Quat>> std::vector<std::shared_ptr<g2o::SE3Quat>> get_poses() {
        std::vector<std::shared_ptr<g2o::VertexSE3Expmap>> ps;

        // Hardcoded two "poses"
        for(int i = 0; i < 2; i++) {
            // This shouldn't be like this
            Eigen::Vector3d trans(i*0.04-1.,0,0);
            Eigen::Quaterniond q;

            q.setIdentity();
            g2o::SE3Quat pose(q, trans);
            std::shared_ptr<g2o::VertexSE3Expmap> p = std::make_shared<g2o::VertexSE3Expmap>();

            p->setId(i);
            p->setFixed(true);
            p->setEstimate(pose);

            optimizer.addVertex(p);
            ps.push_back(pose);            
        }   
    }

    std::vector<std::shared_ptr<g2o::VertexSBAPointXYZ>> get_mappoints(boost::python::list frame, int length) {
        std::vector<std::shared_ptr<g2o::VertexSBAPointXYZ>> mps;

        for(int i = 0; i < length; i++) {         
            boost::python::tuple tuple = boost::python::extract<boost::python::tuple>(frame[i])();
            std::shared_ptr<g2o::VertexSBAPointXYZ> mp = std::make_shared<g2o::VertexSBAPointXYZ>();

            // ORB_SLAM gets the world coordinate instead of camera coordinate
            // maybe we should aswell?
            mp->setId(extract_int(tuple[0]));
            mp->setMarginalized(true);

            // g2o demo adds up gaussian, maybe we should aswell?
            mp->setEstimate(Eigen::Vector3d(
                extract_int(tuple[1]),
                extract_int(tuple[2]),
                extract_int(tuple[3])
            ));

            mps.push_back(mp);
            optimizer.addVertex(mp);
        }

        return mps;
    }
    
    std::make_unique<g2o::SparseOptimizer> optimizer;
};