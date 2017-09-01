/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

#ifndef KARTO_SESYNCSOLVER_H
#define KARTO_SESYNCSOLVER_H

#include <open_karto/Mapper.h>
#include "SESyncOptimizer.h"

/**
 * @brief Wrapper for G2O to interface with Open Karto
 */
class SESyncSolver : public karto::ScanSolver
{

public:

    SESyncSolver();
    
    virtual ~SESyncSolver();
    
    /**
     * @brief Clear the vector of corrections
     * @details Empty out previously computed corrections
     */
    virtual void Clear();
    
    /**
     * @brief Solve the SLAM back-end
     * @details Calls G2O to solve the SLAM back-end
     */
    virtual void Compute();
    
    /**
     * @brief Get the vector of corrections
     * @details Get the vector of corrections
     * @return Vector with corrected poses
     */
    virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

    /**
     * @brief Add a node to pose-graph
     * @details Add a node which is a robot pose to the pose-graph
     * 
     * @param pVertex the node to be added in
     */
    virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
    
    /**
     * @brief Add an edge constraint to pose-graph
     * @details Adds a relative pose measurement constraint between two poses in the graph
     * 
     * @param pEdge [description]
     */
    virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

    /**
     * @brief Get the pose-graph 
     * @details Get the underlying graph from g2o, return the graph of constraints
     * 
     * @param g the graph
     */
    void getGraph(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges);

  private:
    
    karto::ScanSolver::IdPoseVector corrections_;

    SESyncOptimizer optimizer_;

};

#endif // KARTO_SESYNCSOLVER_H

