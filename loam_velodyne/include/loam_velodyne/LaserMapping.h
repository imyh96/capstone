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

#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H


#include "BasicLaserMapping.h"
#include "common.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

////////////////////////////////////////
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include <thread>
using std::thread;
////////////////////////////////////////

namespace loam
{
/** \brief LOAM 레이저 매핑 구성 요소 구현. Implementation of the LOAM laser mapping component.
 *
 */
class LaserMapping : public BasicLaserMapping
{
public:
   explicit LaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief 활성 모드인 구성 요소들을 셋업 / Setup component in active mode.
    *
    * @param node the ROS node handle
    * @param privateNode the private ROS node handle
    */
   virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);


   /** \brief 새 마지막 코너 클라우드에 대한 handler 메소드./ Handler method for a new last corner cloud.
    *
    * @param cornerPointsLastMsg 새로운 last corner cloud 메시지 / the new last corner cloud message
    */
   void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg);

   /** \brief 새 마지막 표면 클라우드에 대한 handler 메소드. / Handler method for a new last surface cloud.
    *
    * @param surfacePointsLastMsg 새로운 last surface cloud 메시지 / the new last surface cloud message
    */
   void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg);

   /** \brief 새 전체 해상도 클라우드에 대한 handler 메소드. / Handler method for a new full resolution cloud.
    *
    * @param laserCloudFullResMsg 새로운 full resolution cloud 메시지 / the new full resolution cloud message
    */
   void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

   /** \brief 새로운 laser odometry를 위한 handler 메소드 / Handler method for a new laser odometry.
    *
    * @param laserOdometry 새로운 laser odometry 메시지 / the new laser odometry message
    */
   void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

   /** \brief IMU 메시지를 위한 handler method. / Handler method for IMU messages.
    *
    * @param imuIn 새로운 IMU 메시지 / the new IMU message
    */
   void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);


   // ////////////////////////////////////////////////////////////////
   void zedPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg);
   void imageLeftRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg);
   void depthHandler(const sensor_msgs::Image::ConstPtr& msg);
   void leftcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg);

   void pclviewerprocess();
   void saveDataSet();
   void loadDataSet();
   void saveDepthSet();
   // ////////////////////////////////////////////////////////////////


   /** \brief 종료될 때까지 루프에서 수신 메시지 처리(활성 모드에서 사용) / Process incoming messages in a loop until shutdown (used in active mode). */
   void spin();


   /** \brief 새롭게 저장된 데이터를 처리하는 메소드. / Try to process buffered data. */
   void process();


protected:
   /** \brief flags 리셋 / Reset flags, etc. */
   void reset();

   /** \brief 새 처리 단계에 필요한 모든 정보를 사용할 수 있는지 확인. / Check if all required information for a new processing step is available. */
   bool hasNewData();

   /** \brief 각 항목을 통해 현재 결과를 publis / Publish the current result via the respective topics. */
   void publishResult();

private:
   ros::Time _timeLaserCloudCornerLast;   ///< 현재 last corner cloud의 시간 / time of current last corner cloud
   ros::Time _timeLaserCloudSurfLast;     ///< 현재 last surface cloud의 시간 / time of current last surface cloud
   ros::Time _timeLaserCloudFullRes;      ///< 현재 full resolution cloud의 시간 / time of current full resolution cloud
   ros::Time _timeLaserOdometry;          ///< 현재 laser odometry의 시간 / time of current laser odometry
   ros::Time _timeZed;//sue 
   bool _newLaserCloudCornerLast;  ///< 새로운 last corner cloud가 들어오면 true / flag if a new last corner cloud has been received
   bool _newLaserCloudSurfLast;    ///< 새로운 last surface cloud가 들어오면 true / flag if a new last surface cloud has been received
   bool _newLaserCloudFullRes;     ///< 새로운 full resolution cloud가 들어오면 true / flag if a new full resolution cloud has been received
   bool _newLaserOdometry;         ///< 새로운 laser odometry가 들어오면 true / flag if a new laser odometry has been received


   nav_msgs::Odometry _odomAftMapped;      ///< 매핑 odometry 메시지 / mapping odometry message
   tf::StampedTransform _aftMappedTrans;   ///< 매핑 odometry 변환 / mapping odometry transformation

   ros::Publisher _pubLaserCloudSurround;    ///< map cloud 메시지 송신기 / map cloud message publisher

   ros::Publisher _pubLaserCloudFullRes;     ///< 현재 full resolution cloud 메시지 송신기 / current full resolution cloud message publisher
   ros::Publisher _pubOdomAftMapped;         ///< 매핑 odometry 송신기 / mapping odometry publisher
   tf::TransformBroadcaster _tfBroadcaster;  ///< 매핑 odometry 변환(행렬) broadcaster / mapping odometry transform broadcaster

   ros::Subscriber _subLaserCloudCornerLast;   ///< last 모서리 cloud 메시지 수신부 / last corner cloud message subscriber
   ros::Subscriber _subLaserCloudSurfLast;     ///< last 평면 cloud 메시지 수신부 / last surface cloud message subscriber
   ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud 메시지 수신부 / full resolution cloud message subscriber
   ros::Subscriber _subLaserOdometry;          ///< laser odometry 메시지 수신부 / laser odometry message subscriber
   ros::Subscriber _subImu;                    ///< IMU 메시지 수신부 / IMU message subscriber

   //////////////////////////////////////////
   ros::Subscriber _subZedTrans;
   ros::Subscriber _subLeftRectified;
   ros::Subscriber _subDepthRectified;
   ros::Subscriber _subLeftcamInfo;
   bool _newLeftcamInfo = false;   // 새 카메라 내부 파라미터가 들어왔는지 확인하는 flag.

   bool _newLeftImg = false;
   bool _newDepthImg = false;

   bool _newZedPose = false;
   bool _newDataSet = false;

   double loaded_zedWorldTrans[6];
   double loaded_transformSum[6];

   cv::Mat loaded_mat_left;
   cv::Mat loaded_mat_depth;
   double loaded_K[9];

   float loaded_depths[1280*720];

   bool dataloaded = false;

   bool saveKvalue = false;
   //////////////////////////////////////////
};

} // end namespace loam

#endif //LOAM_LASERMAPPING_H
