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

#include "loam_velodyne/TransformMaintenance.h"

namespace loam
{

TransformMaintenance::TransformMaintenance()
{
   // initialize odometry and odometry tf messages
   _laserOdometry2.header.frame_id = "/camera_init";
   _laserOdometry2.child_frame_id = "/camera";

   _laserOdometryTrans2.frame_id_ = "/camera_init";
   _laserOdometryTrans2.child_frame_id_ = "/camera";
}


bool TransformMaintenance::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
{
   // advertise integrated laser odometry topic.      // 축적한 laser 주행궤적 토픽을 advertise한다.
   _pubLaserOdometry2 = node.advertise<nav_msgs::Odometry>("/integrated_to_init", 5);

   // subscribe to laser odometry and mapping odometry topics.
   // laser odometry 와 mapping odometry 토픽들을 subscribe하고, 각각의 핸들러 메소드로 보낸다.
   _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &TransformMaintenance::laserOdometryHandler, this);

   _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &TransformMaintenance::odomAftMappedHandler, this);
   
   _subLaserCloudMap = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_3", 2, &TransformMaintenance::laserCloudMapHandler, this);

   return true;
}

void TransformMaintenance::laserCloudMapHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMap)
{
   // _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;   // 위와 동일하게 처리
   laserCloudMap().clear();
   pcl::fromROSMsg(*laserCloudMap, laserCloudMap());
   _newLaserCloudMap = true;
}

void TransformMaintenance::spin()
{
   ros::Rate rate(100);        /// 회전 비율
   bool status = ros::ok();    // ros 상태 체크 

   // loop until shutdown
   while (status)
   {
   ros::spinOnce();
   
   // try processing new data
   process();   

   status = ros::ok();
   rate.sleep();
   }

   /////////// Save Point cloud ////////////
   // 종료시 축적한 포인트 클라우드를 저장한다.
   for (int i = 0; i < PCNUM; i++)
   {
      *_laserCloudSurround += *_laserCloudMapArray[i];
   }
   pcl::io::savePLYFileBinary("/home/cgvlab/ply_test2/output_zedTrans103_from_transMainte.ply", *_laserCloudSurround);
   /////////////////////////////////////////
}

void TransformMaintenance::process()
{
   if (!_newLaserCloudMap) // 새로운 데이타가 도착할때까지 기다림 / waiting for new data to arrive...
      return;

   _newLaserCloudMap = false;   // reset flags, etc.

   // 점들 나눠서 저장하기.
   BasicTransformMaintenance::process();
   
}


void TransformMaintenance::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
   double roll, pitch, yaw;
   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   updateOdometry(-pitch, -yaw, roll,
      laserOdometry->pose.pose.position.x,
      laserOdometry->pose.pose.position.y,
      laserOdometry->pose.pose.position.z);

   transformAssociateToMap();

   geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformMapped()[2], -transformMapped()[0], -transformMapped()[1]);

   _laserOdometry2.header.stamp = laserOdometry->header.stamp;
   _laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
   _laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
   _laserOdometry2.pose.pose.orientation.z = geoQuat.x;
   _laserOdometry2.pose.pose.orientation.w = geoQuat.w;
   _laserOdometry2.pose.pose.position.x = transformMapped()[3];
   _laserOdometry2.pose.pose.position.y = transformMapped()[4];
   _laserOdometry2.pose.pose.position.z = transformMapped()[5];
   _pubLaserOdometry2.publish(_laserOdometry2);

   _laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
   _laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
   _laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped()[3], transformMapped()[4], transformMapped()[5]));
   _tfBroadcaster2.sendTransform(_laserOdometryTrans2);
}


void TransformMaintenance::odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
   double roll, pitch, yaw;
   geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   updateMappingTransform(-pitch, -yaw, roll,
      odomAftMapped->pose.pose.position.x,
      odomAftMapped->pose.pose.position.y,
      odomAftMapped->pose.pose.position.z,

      odomAftMapped->twist.twist.angular.x,
      odomAftMapped->twist.twist.angular.y,
      odomAftMapped->twist.twist.angular.z,

      odomAftMapped->twist.twist.linear.x,
      odomAftMapped->twist.twist.linear.y,
      odomAftMapped->twist.twist.linear.z);
}

} // end namespace loam
