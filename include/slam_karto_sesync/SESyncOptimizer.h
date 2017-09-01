/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

#include <eigen>
#include "SESync.h"

using namespace SESync;

/**
 * @brief A class that wraps SESync functionalities
 * @details Stores the relative pose measurements and solves the pose-graph SLAM problem using SE-Sync
 * 
 */
class SESyncOptimizer
{

public:

	/**
	 * @brief Constructor
	 */
    SESyncOptimizer(){}

    /**
     * @brief Destructor
     */
    ~SESyncOptimizer(){}

    /**
     * @brief Add a new relative pose measurement
     * @details Feed a new relative pose measurement and store it.
     * 
     * @param sourceNode source node of edge
     * @param targetNode target node of edge
     * @param z relative pose measurement
     * @param info information matrix of the relative pose estimate
     */
    void addRelativePoseMeasurement(const int sourceNode, const int targetNode, const Eigen::Vector3d z, const Eigen::Matrix<double,3,3> info);

    /**
     * @brief Solve the optimization problem
     * @details Solve the optimization problem
     */
    void solve();

    /**
     * @brief Get the set of estimated robot poses
     * @details Get the set of estimated robot poses
     */
    void getPoses(std::vector<Eigen::Vector3d> &poses);

private:

	std::vector<SESync::RelativePoseMeasurement> measurements_; //vector of relative pose measurements

	std::vector<Eigen::Vector3d> estimate_;// the estimated robot pose

}