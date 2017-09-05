/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

#include <limits>
#include "SESyncSolver.h"
#include <open_karto/Karto.h>
#include <ros/console.h>


SESyncSolver::SESyncSolver()
{

}

SESyncSolver::~SESyncSolver()
{

}

void SESyncSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector& SESyncSolver::GetCorrections() const
{
  return corrections_;
}

void SESyncSolver::Compute()
{
  corrections_.clear();

  graphNodes_.clear();

  ROS_WARN("[sesync] Solving for loop closure.");
  
  // Do the graph optimization
  optimizer_.solve();

  std::vector<Eigen::Vector3d> poses;

  optimizer_.getPoses(poses);
   
  for (int i = 0; i < poses.size(); i++)
  {
    
    Eigen::Vector3d v = poses[i];

    karto::Pose2 p(v(0),v(1),v(2));
    
    corrections_.push_back(std::make_pair(i, p));   

    graphNodes_.push_back(Eigen::Vector2d(v(0), v(1)));

  }
}

void SESyncSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{

  karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();

  graphNodes_.push_back(Eigen::Vector2d(odom.GetX(), odom.GetY()));
  
}

void SESyncSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{

  
  // Set source and target
  int sourceID = pEdge->GetSource()->GetObject()->GetUniqueId();
  
  int targetID = pEdge->GetTarget()->GetObject()->GetUniqueId();
  
  // Set the measurement (poseGraphEdge distance between vertices)
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());
  
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  
  Eigen::Vector3d z(diff.GetX(), diff.GetY(), diff.GetHeading());
    
  // Set the covariance of the measurement
  karto::Matrix3 covMatrix = pLinkInfo->GetCovariance();
  
  Eigen::Matrix<double,3,3> cov;
  
  cov(0,0) = covMatrix(0,0);
  
  cov(0,1) = cov(1,0) = covMatrix(0,1);
  
  cov(0,2) = cov(2,0) = covMatrix(0,2);
  
  cov(1,1) = covMatrix(1,1);
  
  cov(1,2) = cov(2,1) = covMatrix(1,2);
  
  cov(2,2) = covMatrix(2,2);
  
  // Add the constraint to the optimizer
  ROS_WARN("[sesync] Adding Edge from node %d to node %d.", sourceID, targetID);
  
  optimizer_.addRelativePoseMeasurement(sourceID, targetID, z, cov);

}

void SESyncSolver::getGraph(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges)
{

  nodes = graphNodes_;

  //  for (int i = 0; i < poses.size(); i++)
  // {
    
  //   Eigen::Vector2d v = poses[i];

  //   Eigen::Vector2d p(v(0), v(1));

  //   nodes.push_back(p);
    
  // }

  // double *data1 = new double[3];

  // double *data2 = new double[3];

  // for (SparseOptimizer::EdgeSet::iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it) 
  // {

  //   EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
    
  //   if(e) 
  //   {
      
  //     VertexSE2* v1 = dynamic_cast<VertexSE2*>(e->vertices()[0]);

  //     v1->getEstimateData(data1);

  //     Eigen::Vector2d poseFrom(data1[0], data1[1]);

  //     VertexSE2* v2 = dynamic_cast<VertexSE2*>(e->vertices()[1]);

  //     v2->getEstimateData(data2);

  //     Eigen::Vector2d poseTo(data2[0], data2[1]);

  //     edges.push_back(std::make_pair(poseFrom, poseTo));

  //   }

  // }

  // delete data1;

  // delete data2;

}