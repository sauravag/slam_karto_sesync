/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

#include "SESyncOptimizer"


void SESyncOptimizer::addRelativePoseMeasurement(const int sourceNode, const int targetNode, const Eigen::Vector3d z, const Eigen::Matrix<double,3,3> info)
{
	
	using namespace SESync;

	// generate a SESync relative pose measurement
	SESync::RelativePoseMeasurement z_sesync;

	z_sesync.i = sourceNode;

	z_sesync.j = targetNode;

	z_sesync.t = Eigen::Vector2d(z(0),z(1));

	z_sesync.R = Eigen::Rotation2Dd(z(2)).toRotationMatrix();

	Eigen::Matrix2d tranCov;

	tranCov<< info(1,1),info(1,2), info(1,2), info(2,2);

	z_sesync.tau = 2 / tranCov.inverse().trace();
	
	z_sync.kappa = info(3,3);

	// add to storage
	measurements_.push_back(z_sesync);

	
}

void SESyncOptimizer::solve()
{

	using namespace SESync;

	// setup optimization params
	SESyncOpts opts;

	// solve problem
	SESyncResult result = SESync::SESync(measurements_, opts);

	// move data from results into estimate_ container
	Eigen::MatrixXd rotations = results.Rhat;

	Eigen::MatrixXd translations = results.that;

	std::cout<<"SESyncOptimizer: Rotations = "<<rotations<<std::endl;

	std::cout<<"SESyncOptimizer: Translations = "<<translations<<std::endl;


}

void SESyncOptimizer::getPoses(std::vector<Eigen::Vector3d> &poses)
{
	poses = estimate_;
}
