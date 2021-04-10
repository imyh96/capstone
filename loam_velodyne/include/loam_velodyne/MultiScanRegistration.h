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

#ifndef LOAM_MULTISCANREGISTRATION_H
#define LOAM_MULTISCANREGISTRATION_H


#include "loam_velodyne/ScanRegistration.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cmath>
#include <queue>
#include <set>
#include <string>


namespace loam {


/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring. 
 *  포인트의 세로 각도를 그에 대응하는 라이다의 layer와 선형매핑하기 위한 클래스. 여러가지 라이다의 종류에 따라 그 scan범위(각도)와 layer갯수가 다르므로 이를 세팅해주기 위함.
 */
class MultiScanMapper 
{
public:
  /** \brief Construct a new multi scan mapper instance. // 이 클래스의 생성자.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  MultiScanMapper(const float& lowerBound = -15,
                  const float& upperBound = 15,
                  const uint16_t& nScanRings = 16);


  // 이 클래스의 get 메소드들.
  const float& getLowerBound() { return _lowerBound; }
  const float& getUpperBound() { return _upperBound; }
  const uint16_t& getNumberOfScanRings() { return _nScanRings; }



  /** \brief Set mapping parameters. // set 메소드
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  void set(const float& lowerBound,
           const float& upperBound,
           const uint16_t& nScanRings);



  /** \brief Map the specified vertical point angle to its ring ID. // (포인트의) 각도값을 (라디안으로) 입력하면 그에 대응하는 layer 아이디를 리턴해주는 메소드
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID.                                           // 리턴값.
   */
  int getRingForAngle(const float& angle);



  /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
  static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); }; // 벨로다인 퍽용 생성자.

  /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };


private:
  float _lowerBound;      ///< the vertical angle of the first scan ring
  float _upperBound;      ///< the vertical angle of the last scan ring
  uint16_t _nScanRings;   ///< number of scan rings
  float _factor;          ///< linear interpolation factor. 선형 매핑을 위한 선형 보간 factor.
};



/** \brief Class for registering point clouds received from multi-laser lidars.
 *  여러개의 layer를 가진 라이다로 부터 얻어지는 포인트 클라우드를 등록하기 위한 클래스
 */
class MultiScanRegistration : virtual public ScanRegistration 
{
public:
  MultiScanRegistration(const MultiScanMapper& scanMapper = MultiScanMapper()); // 생성자


  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode); // 셋업 메소드.


  /** \brief Handler method for input cloud messages.             // 들어오는 클라우드 메세지의 handler 메소드
   * 
   * @param laserCloudMsg the new input cloud message to process. // 처리(process)해야 하는 새로 입력된 클라우드 메세지
   */
  void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);


private:
  /** \brief Setup component in active mode.             // 액티브 모드의 컴포넌트들을 setup하는 메소드
   *
   * @param node the ROS node handle 
   * @param privateNode the private ROS node handle
   */
  bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out) override;


  /** \brief Process a new input cloud.                   // 새로 입력된 클라우드를 처리(process) 하기위한 메소드
   *
   * @param laserCloudIn the new input cloud to process.  // 새로 입력된 포인트 클라우드
   * @param scanTime the scan (message) timestamp.        // 그것이 scan된 시간.
   */
  void process(const pcl::PointCloud<pcl::PointXYZI>& laserCloudIn, const Time& scanTime);


private:
  int _systemDelay = 20;             ///< system startup delay counter
  MultiScanMapper _scanMapper;  ///< mapper for mapping vertical point angles to scan ring IDs. // 포인트의 세로 각도를 라이다의 layer와 매핑해주기 위한 mapper 클래스.
  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > _laserCloudScans;                               // 스캔된 포인트 클라우드.

  pcl::PointCloud<pcl::PointXYZRGBNormal> _laserCloudSur;
  
  ros::Subscriber _subLaserCloud;   ///< input cloud message subscriber.                        // 입력되는 포인트클라우드 메세지의 subscriber.

};

} // end namespace loam


#endif //LOAM_MULTISCANREGISTRATION_H
