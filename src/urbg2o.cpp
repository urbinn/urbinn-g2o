#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
//#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
//#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include <opencv2/core/core.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include "urbg2o.h"

using namespace std;

const float deltaMono = sqrt(5.991);
const float CameraFx =  718.856;
const float CameraFy = 718.856;
const float CameraCx = 607.1928;
const float CameraCy = 185.2157;
const float InvCameraFx = 1.0f / CameraFx;
const float InvCameraFy = 1.0f / CameraFy;

Eigen::MatrixXd toEigen(const g2o::SE3Quat &SE3)
{
	Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
	return eigMat;
}

cv::Mat UnprojectStereo(double u, double v, double z)
{
	if(z>0)
	{
		const float x = (u-CameraCx) * z * InvCameraFx;
		const float y = (v-CameraCy) * z * InvCameraFy;
		cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

		cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F );
		//return x3Dc;
		return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
	}
	else
		return cv::Mat();
}

int poseOptimization(Eigen::Ref<Eigen::MatrixXd> coords, Eigen::Ref<Eigen::MatrixXd> pose)
{
	Eigen::Matrix4d matrix;
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;

	linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
			g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
	);
	optimizer.setAlgorithm(solver);
	int nInitialCorrespondences=0;

	// Set Frame vertex
	g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
	//vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
	vSE3->setEstimate( g2o::SE3Quat() );
	vSE3->setId(0);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3);

	// Set MapPoint vertices
	const int N = coords.rows();

	cout << "size " << N << std::endl;

	// added mvbOutlier
	vector<bool> mvbOutlier = vector<bool>(N, false);

	vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
	vector<size_t> vnIndexEdgeMono;
	vpEdgesMono.reserve(N);
	vnIndexEdgeMono.reserve(N);

	// vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
	// vector<size_t> vnIndexEdgeStereo;
	// vpEdgesStereo.reserve(N);
	// vnIndexEdgeStereo.reserve(N);

	const float deltaMono = sqrt(5.991);
	// const float deltaStereo = sqrt(7.815);


	{

		for(int i=0; i<N; i++)
		{
			nInitialCorrespondences++;

			//pFrame->mvbOutlier[i] = false;

			Eigen::Matrix<double,2,1> obs;
			obs[0] = coords(i, 4);
			obs[1] = coords(i, 5);

			g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
			e->setMeasurement(obs);
			e->setInformation(Eigen::Matrix2d::Identity());

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(deltaMono);

			e->fx = CameraFx;
			e->fy = CameraFy;
			e->cx = CameraCx;
			e->cy = CameraCy;
			cv::Mat Xw = UnprojectStereo(coords(i, 1), coords(i, 2), coords(i, 3));

			e->Xw[0] = Xw.at<float>(0, 0);
			e->Xw[1] = Xw.at<float>(1, 0);
			e->Xw[2] = Xw.at<float>(2, 0);

			optimizer.addEdge(e);

			vpEdgesMono.push_back(e);
			vnIndexEdgeMono.push_back(i);
		}
	}


	if(nInitialCorrespondences<3) {
		cout << "initialCorrespondeces < 3";
		//return cv::Mat(1, 1, CV_32F, { 0.0 } );
	}

	cout << "verder";

	// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
	// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
	const float chi2Mono[4]={5.991,5.991,5.991,5.991};
	const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
	const int its[4]={10,10,10,10};    

	int nBad=0;
	for(size_t it=0; it<4; it++)
	{

		vSE3->setEstimate( g2o::SE3Quat() );
		optimizer.initializeOptimization(0);
		optimizer.optimize(its[it]);

		nBad=0;

		cout << "vpEdgesMono" << vpEdgesMono.size() << std::endl;

		for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
		{
			g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

			const size_t idx = vnIndexEdgeMono[i];

			if(mvbOutlier[idx])
			{
				e->computeError();
			}

			const float chi2 = e->chi2();

			if(chi2>chi2Mono[it])
			{                
				mvbOutlier[idx]=true;
				e->setLevel(1);
				nBad++;
			}
			else
			{
				mvbOutlier[idx]=false;
				e->setLevel(0);
			}

			if(it==2)
				e->setRobustKernel(0);
		}
	}
	// Recover optimized pose and return number of inliers
	int numberGoodPoints = nInitialCorrespondences-nBad;
	cout << "number of remaining points " << numberGoodPoints << std::endl;
	g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	pose = SE3quat_recov.to_homogeneous_matrix();
	return numberGoodPoints;
}

