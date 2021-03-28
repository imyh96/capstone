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

#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/common.h"

namespace loam
{

LaserMapping::LaserMapping(const float& scanPeriod, const size_t& maxIterations)
{
   //메핑 odometry와 odmoetry tf의 메시지를 초기화 / initialize mapping odometry and odometry tf messages
   _odomAftMapped.header.frame_id = "/camera_init";
   _odomAftMapped.child_frame_id = "/aft_mapped";

   _aftMappedTrans.frame_id_ = "/camera_init";
   _aftMappedTrans.child_frame_id_ = "/aft_mapped";
}


bool LaserMapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
   // 레이저 매핑 파라미터를 가져옴 / fetch laser mapping params
   float fParam;
   int iParam;

   // 4개의 기본 parameter와 3개의 down sizing 필터 변수를 세팅.
   if (privateNode.getParam("scanPeriod", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setScanPeriod(fParam);
         ROS_INFO("Set scanPeriod: %g", fParam);
      }
   }

   if (privateNode.getParam("maxIterations", iParam))
   {
      if (iParam < 1)
      {
         ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
         return false;
      }
      else
      {
         setMaxIterations(iParam);
         ROS_INFO("Set maxIterations: %d", iParam);
      }
   }

   if (privateNode.getParam("deltaTAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaTAbort(fParam);
         ROS_INFO("Set deltaTAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("deltaRAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaRAbort(fParam);
         ROS_INFO("Set deltaRAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("cornerFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid cornerFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterCorner().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set corner down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("surfaceFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid surfaceFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterSurf().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set surface down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("mapFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid mapFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterMap().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set map down size filter leaf size: %g", fParam);
      }
   }

   // advertise laser mapping topics. // 다른 node로 보낼 topic들 3가지를 advertise.
   _pubLaserCloudSurround = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);      // 월드 좌표계에 축적된 포인트 클라우드
   _pubLaserCloudFullRes  = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2); // 라이다 좌표계의 현재 sweep의 포인트 클라우드
   _pubOdomAftMapped      = node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);              // mapping된 이후의 odometry.

   // subscribe to laser odometry topics. // laser odometry 노드에서 오는 토픽들을 subscribe.
   _subLaserCloudCornerLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_corner_last", 2, &LaserMapping::laserCloudCornerLastHandler, this);     // subscribe한 모든 데이터들은 바로 각각의 핸들러로 보내진다.

   _subLaserCloudSurfLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_surf_last", 2, &LaserMapping::laserCloudSurfLastHandler, this);

   _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &LaserMapping::laserOdometryHandler, this);

   _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_3", 2, &LaserMapping::laserCloudFullResHandler, this);

   // subscribe to IMU topic. // Multi Scan registration 노드에서 IMU 토픽을 subscribe.
   _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &LaserMapping::imuHandler, this);

   return true;
}


void LaserMapping::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg)
{
   _timeLaserCloudCornerLast = cornerPointsLastMsg->header.stamp; // time stamp.
   laserCloudCornerLast().clear();                                // 이전 데이터 정리.
   pcl::fromROSMsg(*cornerPointsLastMsg, laserCloudCornerLast()); // subscribe해 온 msg를 _laserCloudCornerLast 저장
   _newLaserCloudCornerLast = true;                               // 새로운 데이터가 입력 되었음을 나타내는 flag를 ON.
}

void LaserMapping::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg)
{
   _timeLaserCloudSurfLast = surfacePointsLastMsg->header.stamp;  // 위와 동일하게 처리
   laserCloudSurfLast().clear();
   pcl::fromROSMsg(*surfacePointsLastMsg, laserCloudSurfLast());
   _newLaserCloudSurfLast = true;
}

void LaserMapping::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
   _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;   // 위와 동일하게 처리
   laserCloud().clear();
   pcl::fromROSMsg(*laserCloudFullResMsg, laserCloud());
   _newLaserCloudFullRes = true;
}

void LaserMapping::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
   _timeLaserOdometry = laserOdometry->header.stamp;   // time stamp.

   double roll, pitch, yaw;
   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;   // 받은 메세지와 동일한 형식으로 저장.
   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw); // 그것을 3x3 행렬 형식으로 바꿔 메세지의 rotation 정보를 얻는다.

   updateOdometry(-pitch, -yaw, roll,
                  laserOdometry->pose.pose.position.x,
                  laserOdometry->pose.pose.position.y,
                  laserOdometry->pose.pose.position.z);     // _transformSum 변수에 subscribe한 메세지의 rotation, translation 정보를 업데이트.

   _newLaserOdometry = true;
}

void LaserMapping::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
   double roll, pitch, yaw;
   tf::Quaternion orientation;
   tf::quaternionMsgToTF(imuIn->orientation, orientation);  // 받은 메세지를 tf::Quaternion 형식으로 저장.
   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);     // 그것을 3x3 행렬 형식으로 바꿔 메세지의 rotation 정보를 얻는다.
   updateIMU({ fromROSTime(imuIn->header.stamp) , roll, pitch }); // 해당 정보를 _imuHistory에 새로운 imu state로써 업데이트.
}


void LaserMapping::spin()
{
   ros::Rate rate(100);
   bool status = ros::ok();

   while (status)
   {
      ros::spinOnce();

      // try processing buffered data.
      process();

      status = ros::ok();
      rate.sleep();
   }
}

void LaserMapping::reset()
{
   _newLaserCloudCornerLast = false;
   _newLaserCloudSurfLast = false;
   _newLaserCloudFullRes = false;
   _newLaserOdometry = false;
}

bool LaserMapping::hasNewData()
{
   return _newLaserCloudCornerLast && _newLaserCloudSurfLast &&
      _newLaserCloudFullRes && _newLaserOdometry &&
      fabs((_timeLaserCloudCornerLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudSurfLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeLaserOdometry).toSec()) < 0.005;
}

void LaserMapping::process()
{
   if (!hasNewData()) // 새로운 데이타가 도착할때까지 기다림 / waiting for new data to arrive...
      return;

   reset();// reset flags, etc.

   if (!BasicLaserMapping::process(fromROSTime(_timeLaserOdometry)))
      return;

   publishResult();
}

void LaserMapping::publishResult()
{
   // publish new map cloud according to the input output ratio.     // 입력, 출력비에 따라 새로운 map cloud를 publish.
   if (hasFreshMap()) // publish new map cloud.                      // map cloud가 새롭게 업데이트 되었다면 publish 한다.
      publishCloudMsg(_pubLaserCloudSurround, laserCloudSurroundDS(), _timeLaserOdometry, "/camera_init");

   // publish transformed full resolution input cloud.               // 변환된 full resolution 포인트 클라우드를 publish.
   publishCloudMsg(_pubLaserCloudFullRes, laserCloud(), _timeLaserOdometry, "/camera_init");


   // publish odometry after mapped transformations.                 // 맵을 업데이트 한 후의 현재까지의 odometry 정보를 publish.
   geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
   (transformAftMapped().rot_z.rad(), -transformAftMapped().rot_x.rad(), -transformAftMapped().rot_y.rad());

   _odomAftMapped.header.stamp = _timeLaserOdometry;
   _odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
   _odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
   _odomAftMapped.pose.pose.orientation.z = geoQuat.x;
   _odomAftMapped.pose.pose.orientation.w = geoQuat.w;
   _odomAftMapped.pose.pose.position.x = transformAftMapped().pos.x();
   _odomAftMapped.pose.pose.position.y = transformAftMapped().pos.y();
   _odomAftMapped.pose.pose.position.z = transformAftMapped().pos.z();
   _odomAftMapped.twist.twist.angular.x = transformBefMapped().rot_x.rad();
   _odomAftMapped.twist.twist.angular.y = transformBefMapped().rot_y.rad();
   _odomAftMapped.twist.twist.angular.z = transformBefMapped().rot_z.rad();
   _odomAftMapped.twist.twist.linear.x = transformBefMapped().pos.x();
   _odomAftMapped.twist.twist.linear.y = transformBefMapped().pos.y();
   _odomAftMapped.twist.twist.linear.z = transformBefMapped().pos.z();
   _pubOdomAftMapped.publish(_odomAftMapped);


   // 위와 같은 정보를 tf로 broadcast.
   _aftMappedTrans.stamp_ = _timeLaserOdometry;
   _aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
   _aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped().pos.x(),
                                         transformAftMapped().pos.y(),
                                         transformAftMapped().pos.z()));
   _tfBroadcaster.sendTransform(_aftMappedTrans);
}

} // end namespace loam
