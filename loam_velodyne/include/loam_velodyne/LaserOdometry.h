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

#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H


#include "Twist.h"
#include "nanoflann_pcl.h"

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "BasicLaserOdometry.h"

// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CameraInfo.h>
// #include "time_utils.h"

namespace loam
{

  /** \brief Implementation of the LOAM laser odometry component.
   * 롬에서의 레이저 주행거리 측정 컴포넌트의 구현을 위한 클래스.
   */
  class LaserOdometry : public BasicLaserOdometry
  {
  public:
    explicit LaserOdometry(float scanPeriod = 0.1, uint16_t ioRatio = 1, size_t maxIterations = 25); //iteration: 반복  //uint16_t ioRatio = 2 -> 1


    /************* 셋팅 함수 ***************/    
    /** \brief Setup component. 요소 세팅
     *
     * @param node the ROS node handle   일반 노드
     * @param privateNode the private ROS node handle  private 노드
     */
    virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

    // ////////////////////////////////////////
    // void imageLeftRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg);
    // void depthHandler(const sensor_msgs::Image::ConstPtr& msg);
    // void leftcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg);
    // ////////////////////////////////////////

    // 핸들러 = subscribe 해 온 메세지를 이 노드의 적절한 변수로 잘 저장하기 위한 메소드들. 각각이 취하는 행동에는 큰 차이 없다.

    /************* sharp corner 특징점 클라우드 핸들러 ***************/
    /** \brief Handler method for a new sharp corner cloud.
     *
     * @param cornerPointsSharpMsg the new sharp corner cloud message. sharp corner 포인트 클라우드 
     */
    void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg);

    /************* less sharp corner 특징점 클라우드 핸들러 ***************/
    /** \brief Handler method for a new less sharp corner cloud.
     *
     * @param cornerPointsLessSharpMsg the new less sharp corner cloud message
     */
    void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg);

    /************* flat surface 특징점 클라우드 핸들러 ***************/
    /** \brief Handler method for a new flat surface cloud.
     *
     * @param surfPointsFlatMsg the new flat surface cloud message
     */
    void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg);

    /************* less flat surface 특징점 클라우드 핸들러 ***************/
    /** \brief Handler method for a new less flat surface cloud.
     *
     * @param surfPointsLessFlatMsg the new less flat surface cloud message
     */
    void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg);

    /************* full 해상도 포인트 클라우드 핸들러 ***************/
    /** \brief Handler method for a new full resolution cloud.
     *
     * @param laserCloudFullResMsg the new full resolution cloud message
     */
    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

    /************* IMU transform 정보 핸들러 ***************/
    /** \brief Handler method for a new IMU transformation information.
     *
     * @param laserCloudFullResMsg the new IMU transformation information message
     */
    void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg);


    /** \brief Process incoming messages in a loop until shutdown (used in active mode). */ 
    // laser_odometry_node.cpp 에서 node에 데이터가 있을경우 spin 유지
    void spin();


    /** \brief Try to process buffered data. */
    void process();     // 새로운 데이터가 있을경우 reset함수 실행하고 BasiclaserOdometry.cpp의 process를 실행
                        // 새로운 데이터가 없을경우 리턴함.  -> spin에 포함됨

  protected:
    /** \brief Reset flags, etc. */
    void reset();       // 데이터를 리셋하는것이 아니라 새로운 데이터가 있는지 없는지를 check하는 부분을 reset함
                        // 즉 hasNewData에서 true로 표시된 부분을 false로 바꿔줌 -> 데이터가 사용되었으니 새로운데이터로 바꿔도 된다고 바꿔주는 과정

    /** \brief Check if all required information for a new processing step is available. */
    bool hasNewData();   // 새로운 데이터가 들어오면 true  아니면 false

    // 처리된 데이터들을 퍼블리쉬 해 주는 메소드 
    /** \brief Publish the current result via the respective topics. */
    void publishResult();

  private:
    uint16_t _ioRatio;       ///< ratio of input to output frames.    // laser mapping node의 Hz를 1Hz로 낮추는 원인중 하나.

// 각 데이터들의 시간값.
    ros::Time _timeCornerPointsSharp;      ///< time of current sharp corner cloud
    ros::Time _timeCornerPointsLessSharp;  ///< time of current less sharp corner cloud
    ros::Time _timeSurfPointsFlat;         ///< time of current flat surface cloud
    ros::Time _timeSurfPointsLessFlat;     ///< time of current less flat surface cloud
    ros::Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
    ros::Time _timeImuTrans;               ///< time of current IMU transformation information

// 새로운 데이터가 입력되었는지 확인하기 위한 flag들. 새로운 데이터가 입력된 경우 true가 됨.
    bool _newCornerPointsSharp;       ///< flag if a new sharp corner cloud has been received
    bool _newCornerPointsLessSharp;   ///< flag if a new less sharp corner cloud has been received
    bool _newSurfPointsFlat;          ///< flag if a new flat surface cloud has been received
    bool _newSurfPointsLessFlat;      ///< flag if a new less flat surface cloud has been received
    bool _newLaserCloudFullRes;       ///< flag if a new full resolution cloud has been received
    bool _newImuTrans;                ///< flag if a new IMU transformation information cloud has been received

//message, publisher, subscriber 선언
    nav_msgs::Odometry _laserOdometryMsg;       ///< laser odometry message
    tf::StampedTransform _laserOdometryTrans;   ///< laser odometry transformation. 라이다 이동기록에 대한 변환행렬

    ros::Publisher _pubLaserCloudCornerLast;  ///< last corner cloud message publisher
    ros::Publisher _pubLaserCloudSurfLast;    ///< last surface cloud message publisher
    ros::Publisher _pubLaserCloudFullRes;     ///< full resolution cloud message publisher
    ros::Publisher _pubLaserOdometry;         ///< laser odometry publisher.        퍼블리쉬 하기위한 퍼블리셔 선언
    tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster

    ros::Subscriber _subCornerPointsSharp;      ///< sharp corner cloud message subscriber
    ros::Subscriber _subCornerPointsLessSharp;  ///< less sharp corner cloud message subscriber
    ros::Subscriber _subSurfPointsFlat;         ///< flat surface cloud message subscriber
    ros::Subscriber _subSurfPointsLessFlat;     ///< less flat surface cloud message subscriber
    ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud message subscriber
    ros::Subscriber _subImuTrans;               ///< IMU transformation information message subscriber
    
    ///////////////////////////////////
  };

} // end namespace loam

#endif //LOAM_LASERODOMETRY_H
