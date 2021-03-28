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

#ifndef LOAM_SCANREGISTRATION_H
#define LOAM_SCANREGISTRATION_H


#include "common.h"

#include <stdint.h>

#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

#include "BasicScanRegistration.h"


namespace loam
{
  /** \brief Base class for LOAM scan registration implementations. 롬 스캔 등록을 위한 기본 클래스.
   *
   * As there exist various sensor devices, producing differently formatted point clouds,
   * specific implementations are needed for each group of sensor devices to achieve an accurate registration.
   * This class provides common configurations, buffering and processing logic.
   * 
   * 여러가지 다른 종류의 라이다가 다른 형식의 포인트 클라우드를 생성하므로 정확한 등록을 위해선 그에 맞춘 구현이 필요. 이 클래스는 그 중 공통된 구성, 버퍼링, 처리 로직만을 제공함.
   */
  class ScanRegistration : protected BasicScanRegistration
  {
  public:

    /** \brief Setup component. 컴포넌트들의 셋업.
     *
     * @param node the ROS node handle
     * @param privateNode the private ROS node handle
     */
    virtual bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out);


    /** \brief Handler method for IMU messages. // IMU 센서로 부터의 메세지를 처리하기 위한 핸들러.
     *
     * @param imuIn the new IMU message.        // 새로운 IMU 메세지.
     */
    virtual void handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn);


  protected:
    /** \brief Publish the current result via the respective topics. */
    // 현재 결과를 각각의 토픽을 통해 퍼블리쉬 해주는 메소드. 
    // multi_scan_registration_node에서 퍼블리쉬 하는 총 6가지 메세지를 퍼블리쉬함.
    void publishResult();


  private:
    /** \brief Parse node parameter. 노드의 매개변수를 파싱하는 메소드. 모든 매개변수들이 유효한 경우 true를 리턴, 아닌 경우 false를 리턴한다.
    *
    * @param nh the ROS node handle
    * @return true, if all specified parameters are valid, false if at least one specified parameter is invalid
    */
    bool parseParams(const ros::NodeHandle& nh, RegistrationParams& config_out);


  private:
    

    ros::Subscriber _subImu;                    ///< IMU message subscriber.                    // IMU센서로부터의 값을 받아오는 subscriber

    // ros::Publisher _pubPixelCloud;    // 새로 추가.
    
    ros::Publisher _pubLaserCloud;              ///< full resolution cloud message publisher.   // 아래 6개는 다른 node로 값을 주기 위한 총 6가지 메세지의 publisher.
    ros::Publisher _pubCornerPointsSharp;       ///< sharp corner cloud message publisher
    ros::Publisher _pubCornerPointsLessSharp;   ///< less sharp corner cloud message publisher
    ros::Publisher _pubSurfPointsFlat;          ///< flat surface cloud message publisher
    ros::Publisher _pubSurfPointsLessFlat;      ///< less flat surface cloud message publisher
    ros::Publisher _pubImuTrans;                ///< IMU transformation message publisher

    ros::Publisher _pubRightRectified;
    ros::Publisher _pubLeftRectified;
  };

} // end namespace loam


#endif //LOAM_SCANREGISTRATION_H
