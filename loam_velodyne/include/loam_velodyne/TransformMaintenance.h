// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#ifndef LOAM_TRANSFORMMAINTENANCE_H
#define LOAM_TRANSFORMMAINTENANCE_H


#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "loam_velodyne/BasicTransformMaintenance.h"

////////////////////////////////////////
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "common.h"

#include <thread>
using std::thread;
////////////////////////////////////////

namespace loam {

/** \brief Implementation of the LOAM transformation maintenance component. 변환(행렬) 유지관리(maintenance) 컴포넌트의 구현을 위한 클래스
 * 
 */
class TransformMaintenance: public BasicTransformMaintenance 
{  
public:
  explicit TransformMaintenance(); // 생성자.


  /** \brief Setup component. // 셋업 메소드.
   *
   * @param node the ROS node handle  // ROS 노드 핸들. 퍼블리쉬를 위한 advertise와 subscribe를, node.advertise<데이터 타입>, node.subscribe<데이터 타입> 형태로 수행할 수 있도록 해 준다.
   * @param privateNode the private ROS node handle   // private ROS 노드 핸들.   일반 노드 핸들과 달리 노드의 namespace를 붙여주지 않아도 parameter에 접근 가능한 노드핸들.
   */
  virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);


  /** \brief Handler method for laser odometry messages. // laser odometry 메세지의 핸들러 메소드.
   *
   * @param laserOdometry the new laser odometry. // 새로 subscribe한 laser odometry 메세지.
   */
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);


  /** \brief Handler method for mapping odometry messages. // mapping odometry 메세지의 핸들러 메소드.
   *
   * @param odomAftMapped the new mapping odometry. // 새로 subscribe한 mapping odometry 메세지.
   */
  void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);

  ////////////////////////////////////////
  void laserCloudMapHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud);
  void spin();
  void process();
  ////////////////////////////////////////

private:
  nav_msgs::Odometry _laserOdometry2;         ///< latest integrated laser odometry message.        가장 최근에 통합된 레이저 오도메트리 메세지.
  tf::StampedTransform _laserOdometryTrans2;  ///< latest integrated laser odometry transformation. 가장 최근에 통합된 레이저 오도메트리 변환행렬.

  ros::Publisher _pubLaserOdometry2;          ///< integrated laser odometry publisher.                  통합된 레이저 오도메트리의 publisher.
  tf::TransformBroadcaster _tfBroadcaster2;   ///< integrated laser odometry transformation broadcaster. 통합된 레이저 오도메트리 변환행렬의 broadcaster.

  ros::Subscriber _subLaserOdometry;    ///< (high frequency) laser odometry subscriber.  레이저 오도메트리의 subscriber.
  ros::Subscriber _subOdomAftMapped;    ///< (low frequency) mapping odometry subscriber. 매핑 오도메트리의 subscriber.

  ////////////////////////////////////////
  ros::Subscriber _subLaserCloud; 
  // thread t1;
  ////////////////////////////////////////
};

} // end namespace loam


#endif //LOAM_TRANSFORMMAINTENANCE_H
