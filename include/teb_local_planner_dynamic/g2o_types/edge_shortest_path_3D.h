/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef EDGE_SHORTEST_PATH_3D_H_
#define EDGE_SHORTEST_PATH_3D_H_

#include <float.h>

#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <teb_local_planner_dynamic/g2o_types/base_teb_edges.h>
#include <teb_local_planner_dynamic/g2o_types/vertex_pose.h>

#include <Eigen/Core>

namespace teb_local_planner {

/**
 * @class EdgeShortestPath
 * @brief Edge defining the cost function for minimizing the Euclidean distance between two consectuive poses.
 *
 * @see TebOptimalPlanner::AddEdgesShortestPath
 */
class EdgeShortestPath3D : public BaseTebMultiEdge<1, double> {
public:
  /**
   * @brief Construct edge.
   */
  EdgeShortestPath3D() {
      //this->setMeasurement(0.);
  }
  EdgeShortestPath3D(int size)
  {
   this->resize(size); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }
  /**
   * @brief Actual cost function
   */
  void computeError() {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeShortestPath3D()");
    _error[0] = 0.0;
    for (unsigned int i=1;i<_vertices.size();++i)
    {
    const VertexPose *pose1 = static_cast<const VertexPose*>(_vertices[i-1]);
    const VertexPose *pose2 = static_cast<const VertexPose*>(_vertices[i]);
    double dt = pose2->t()-pose1->t();
    double space_dist_2 = (cfg_->optim.weight_shortest_path*(pose2->position()-pose1->position())).squaredNorm();
    double t_dist_2 = pow(cfg_->optim.weight_optimaltime*(dt),2);
    _error[0] += sqrt(t_dist_2+space_dist_2);
    }
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeShortestPath::computeError() _error[0]=%f\n", _error[0]);
  }
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace

#endif /* EDGE_SHORTEST_PATH_3D_H_ */
