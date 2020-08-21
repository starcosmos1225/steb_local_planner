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

#ifndef EDGE_TIMELIMIT_H_
#define EDGE_TIMELIMIT_H_

#include <float.h>

#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <teb_local_planner_dynamic/g2o_types/base_teb_edges.h>
#include <teb_local_planner_dynamic/g2o_types/penalties.h>
#include <teb_local_planner_dynamic/teb_config.h>
#include <teb_local_planner_dynamic/g2o_types/vertex_pose.h>
#include <Eigen/Core>

namespace teb_local_planner
{

  
/**
 * @class EdgeTimeOptimal
 * @brief Edge defining the cost function for minimizing transition time of the trajectory.
 * 
 * The edge depends on a single vertex \f$ \Delta T_i \f$ and minimizes: \n
 * \f$ \min \Delta T_i^2 \cdot scale \cdot weight \f$. \n
 * \e scale is determined using the penaltyEquality() function, since we experiences good convergence speeds with it. \n
 * \e weight can be set using setInformation() (something around 1.0 seems to be fine). \n
 * @see TebOptimalPlanner::AddEdgesTimeOptimal
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeTimeLimit : public BaseTebBinaryEdge<1, double, VertexPose,VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */
  EdgeTimeLimit()
  {
    this->setMeasurement(0.);
  }
  
  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeTimeOptimal()");
    const VertexPose* vertex1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* vertex2 = static_cast<const VertexPose*>(_vertices[1]);
    double dt = vertex2->t() - vertex1->t();
   _error[0] = 1000000*penaltyBoundFromBelow(dt, 0, 1e-6);
   //dff_ = _error[0];
//   if (_error[0]<1e-6)
//   {
//       _error[0] = 1000000*(pow(2,100*(1e-6-_error[0]))-1)+1e-6;
//   }
  
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeTimeOptimal::computeError() _error[0]=%f\n",_error[0]);
  }

//#ifdef USE_ANALYTIC_JACOBI
//  /**
//   * @brief Jacobi matrix of the cost function specified in computeError().
//   */
//  void linearizeOplus()
//  {
//    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeTimeOptimal()");
//    if (dff_<1e-6)
//    {
//        _jacobianOplusXi( 0 , 0 ) = -1000000*100*log(2)*pow(2,100*(1e-6-dff_));
//    }else
//        _jacobianOplusXi( 0 , 0 ) = 1;
//  }
//#endif
  
  
public:        
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double dff_;
};

}; // end namespace

#endif /* EDGE_TIMEOPTIMAL_H_ */
