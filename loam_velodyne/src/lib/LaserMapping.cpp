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
   _pubLaserCloudMap = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);      // 월드 좌표계에 축적된 포인트 클라우드
   _pubLaserCloudFullRes  = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2); // 라이다 좌표계의 현재 sweep의 포인트 클라우드
   _pubOdomAftMapped      = node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);              // mapping된 이후의 odometry.

   //*_pubLaserCloudSurroundColor = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround_color", 1);

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
   //_subImu = node.subscribe<sensor_msgs::Imu>("/zed2/zed_node/imu/data", 50, &LaserMapping::imuHandler, this);

   /////////////////////////////////////////////
   _subZedTrans = node.subscribe<geometry_msgs::PoseStamped>("/zed2/zed_node/pose", 10, &LaserMapping::zedPoseHandler, this);   // zed2로 부터 월드 좌표계에서의 센서 위치 받아오기.
   // _subZedTrans = node.subscribe<geometry_msgs::PoseWithCovariance>("/zed2/zed_node/pose_with_covariance", 10, &LaserMapping::zedCovarianceHandler, this);

   _subLeftRectified  = node.subscribe("/zed2/zed_node/left/image_rect_color", 10, &LaserMapping::imageLeftRectifiedHandler, this);
   _subDepthRectified = node.subscribe("/zed2/zed_node/depth/depth_registered", 10, &LaserMapping::depthHandler, this);
   /////////////////////////////////////////////

   if(!_newLeftcamInfo)
      _subLeftcamInfo = node.subscribe("/zed2/zed_node/left/camera_info", 10, &LaserMapping::leftcamInfoHandler, this);

   return true;
}

void LaserMapping::imageLeftRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("Left Rectified image received from ZED - Size: %dx%d",
    //          msg->width, msg->height);

    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }

    _mat_left = cv_ptr->image;
}
void LaserMapping::depthHandler(const sensor_msgs::Image::ConstPtr& msg) {

    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }

    _mat_depth = cv_ptr->image;

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    depths = (float*)(&msg->data[0]);
}

void LaserMapping::leftcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg) {

   K = (cv::Mat_<float>(3,3) <<  msg->P[0], msg->P[1], msg->P[2],
                                 msg->P[4], msg->P[5], msg->P[6],
                                 msg->P[8], msg->P[9], msg->P[10] );

   // 두 행렬 곱하기.
   KE = K * E;

   _newLeftcamInfo = true;   // 한번만 시행되도록 flag ON.
}

void LaserMapping::zedPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    // Camera position in map frame
    double tx = msg->pose.position.x;
    double ty = msg->pose.position.y;
    double tz = msg->pose.position.z;

    // Orientation quaternion
   //  tf2::Quaternion q(
   //      msg->pose.orientation.x,
   //      msg->pose.orientation.y,
   //      msg->pose.orientation.z,
   //      msg->pose.orientation.w);
      tf::Quaternion q(
         msg->pose.orientation.x,
         msg->pose.orientation.y,
         msg->pose.orientation.z,
         msg->pose.orientation.w);

   //  3x3 Rotation matrix from quaternion
   //  tf2::Matrix3x3 m(q);
    tf::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    updateZedPose(pitch, yaw, roll, tx, ty, tz);

   //  // Output the measure
   //  ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
   //           msg->header.frame_id.c_str(),
   //           tx, ty, tz,
   //           roll * (180/M_PI), pitch * (180/M_PI), yaw * (180/M_PI));
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

void LaserMapping::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
   //*
   //std::cout << "Laser mapping's IMU handler." << std::endl;
   //ROS_INFO("Laser mapping's IMU handler.");
   //*
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

   ///////// Point cloud viewer ////////////    // 초기화
   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(laserCloudSurround());
   
   int cnt = 0;   // counts number of point clouds drawn on viewer.
   /////////////////////////////////////////

   while (status)
   {
      ros::spinOnce();

      // try processing buffered data.
      process();


      ///////// Point cloud viewer ////////////
      if(newPointCloud)    // 클라우드가 새로 업데이트 되었을 때만 viewer에 업데이트 해 준다.
      {
         viewer->addPointCloud(laserCloudSurround(), rgb, std::to_string(cnt));
         
         cnt++;
         newPointCloud = false;  // 업데이트를 확인하기 위한 flag.
      }
      viewer->spinOnce();
      /////////////////////////////////////////

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
   // # laserCloudSurround 는 RVIZ로 publish하지 않고 PCL_Visualizer로 확인한다. (용량이 너무 크기 때문)

   if (_newLaserCloudMap)){ // publish new map cloud to transform maintenance node.                      // map cloud가 새롭게 업데이트 되었다면 publish 한다.
      publishCloudMsg(_pubLaserCloudMap, laserCloudMap(), _timeLaserOdometry, "/camera_init");
      _newLaserCloudMap = false;
   }
   

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
