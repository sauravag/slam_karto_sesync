/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

#include "SESyncOptimizer.h"

SESyncOptimizer::SESyncOptimizer()
{
	measurements_.clear();

}

void SESyncOptimizer::addRelativePoseMeasurement(const int sourceNode, const int targetNode, const Eigen::Vector3d z, const Eigen::Matrix<double,3,3> cov)
{
	
	using namespace SESync;

	// generate a SESync relative pose measurement
	SESync::RelativePoseMeasurement z_sesync;

	z_sesync.i = sourceNode;

	z_sesync.j = targetNode;

	z_sesync.t = Eigen::Vector2d(z(0),z(1));

	z_sesync.R = Eigen::Rotation2Dd(z(2)).toRotationMatrix();

	Eigen::Matrix2d tranCov;

	tranCov << cov(0, 0), cov(0,1), cov(0,1), cov(1,1);

	z_sesync.tau = 2.0 / tranCov.trace();
	
	z_sesync.kappa = 1.0 / cov(2,2);

	std::cout<<" tau = " << 2.0 / tranCov.trace() <<"  kapp = "<< 1.0 / cov(2,2) <<std::endl;

	std::cout<<" t = "<<z_sesync.t << "  R = "<<z_sesync.R<<std::endl;
	// add to storage
	measurements_.push_back(z_sesync);
	
}

void SESyncOptimizer::solve()
{

	using namespace SESync;

	estimate_.clear();

	// setup optimization params
	SESyncOpts opts;

	opts.verbose = true; 

	// solve problem
	SESyncResult results = SESync::SESync(measurements_, opts);

	// move data from results into estimate_ container
	Eigen::MatrixXd rotations = results.Rhat;

	Eigen::MatrixXd translations = results.that;

	int numPoses = translations.cols();

	for(int i = 0; i < numPoses; i++)
	{
		double x = translations(0,i);

		double y = translations(1,i);

		Eigen::MatrixXd Rmat = rotations.block(0,2*i,2,2);

		std::cout<<"Rmat = " << Rmat <<std::endl;

		double heading = rotMat2Yaw(Rmat);

		Eigen::Vector3d p(x,y,heading);

		estimate_.push_back(p);
	}

}

void SESyncOptimizer::getPoses(std::vector<Eigen::Vector3d> &poses)
{
	poses = estimate_;
}

double SESyncOptimizer::rotMat2Yaw(Eigen::MatrixXd R)
{
	double c = R(0,0);
	
	double s = R(1,0);

	return atan2(s,c);
}