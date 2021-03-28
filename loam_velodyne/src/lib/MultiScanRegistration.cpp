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


#include "loam_velodyne/MultiScanRegistration.h"
#include "math_utils.h"
#include <pcl_conversions/pcl_conversions.h>


namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{

}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}



int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}






MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper)
    : _scanMapper(scanMapper)
{};



bool MultiScanRegistration::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
  RegistrationParams config;
  if (!setupROS(node, privateNode, config))
    return false;

  configure(config);
  return true;
}

bool MultiScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out)
{
  if (!ScanRegistration::setupROS(node, privateNode, config_out))   // ScanRegistration::setupROS의 리턴값이 false인 경우 false를 리턴하도록.
    return false;

  // fetch scan mapping params // 현재 연결된 라이다 모델을 확인하고 그에 맞는 설정을 진행한다.
  std::string lidarName;

  if (privateNode.getParam("lidar", lidarName)) {
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str());
      return false;
    }

    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());
    if (!privateNode.hasParam("scanPeriod")) {
      config_out.scanPeriod = 0.1;
      ROS_INFO("Set scanPeriod: %f", config_out.scanPeriod);
    }
  } else {    // 만일 \"VLP-16\", \"HDL-32\" \"HDL-64E\" 가 아닌경우 연결된 라이다의 상태를 확인하고 그에 맞는 세팅을 진행한다.
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }
  }
  
  // receiving zed2's right & left image topic
  //_subRightRectified = node.subscribe("/zed2/zed_node/right/image_rect_color", 10, &MultiScanRegistration::imageRightRectifiedHandler, this);
  // _subLeftRectified  = node.subscribe("/zed2/zed_node/left/image_rect_color", 10, &MultiScanRegistration::imageLeftRectifiedHandler, this);
  //_subLeftRectified  = node.subscribe("/zed2/zed_node/left_raw/image_raw_color", 10, &MultiScanRegistration::imageLeftRectifiedHandler, this);

  // receiving zed2's camera calibration data topic
  if(!_newLeftcamInfo)
    _subLeftcamInfo = node.subscribe("/zed2/zed_node/left/camera_info", 10, &MultiScanRegistration::leftcamInfoHandler, this);
  //_subRightcamInfo = node.subscribe("/zed2/zed_node/right/camera_info", 10, &MultiScanRegistration::rightcamInfoCallback, this);


  // receiving zed2's depth image topic
  _subDepthRectified = node.subscribe("/zed2/zed_node/depth/depth_registered", 10, &MultiScanRegistration::depthHandler, this);

  
  // subscribe to input cloud topic   // 해당 라이다 센서로 부터 포인트 클라우드를 받아온다.
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

  return true;
}

// /////////////////////////////////////////////////////
// void MultiScanRegistration::imageLeftRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg) {
//     // ROS_INFO("Left Rectified image received from ZED - Size: %dx%d",
//     //          msg->width, msg->height);

//     cv_bridge::CvImageConstPtr cv_ptr;
//     try{
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e){
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//     }

//     _mat_left = cv_ptr->image;
// }

// void MultiScanRegistration::imageRightRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg) {
//     // ROS_INFO("Right Rectified image received from ZED - Size: %dx%d",
//     //          msg->width, msg->height);
    
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try{
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e){
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//     }

//     _mat_right = cv_ptr->image;
// }

void MultiScanRegistration::depthHandler(const sensor_msgs::Image::ConstPtr& msg) {

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

void MultiScanRegistration::leftcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg) {

    K = (cv::Mat_<float>(3,3) <<  msg->P[0], msg->P[1], msg->P[2],
                                  msg->P[4], msg->P[5], msg->P[6],
                                  msg->P[8], msg->P[9], msg->P[10] );

  
    // 두 행렬 곱하기.
    KE = K * E;

    _newLeftcamInfo = true;   // 한번만 시행되도록 flag ON.
}

void MultiScanRegistration::rightcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    ROS_INFO( "Right camera: \n K matrix: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \n R matrix: %.3f, %.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \n P matrix: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
              msg->K[0], msg->K[1], msg->K[2], msg->K[3], msg->K[4], msg->K[5], msg->K[6], msg->K[7], msg->K[8],
              msg->R[0], msg->R[1], msg->R[2], msg->R[3], msg->R[4], msg->R[5], msg->R[6], msg->R[7], msg->R[8],
              msg->P[0], msg->P[1], msg->P[2], msg->P[3], msg->P[4], msg->P[5], msg->P[6], msg->P[7], msg->P[8], msg->P[9], msg->P[10], msg->P[11]);
}
/////////////////////////////////////////////////////


void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  if (_systemDelay > 0) 
  {
    --_systemDelay;
    return;
  }

  // fetch new input cloud.   // 새로운 input 클라우드를 받아온다.
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  //std::cout << "invKE: " << invKE << std::endl;

  process(laserCloudIn, fromROSTime(laserCloudMsg->header.stamp));
}



void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZI>& laserCloudIn, const Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();

  // determine scan start and end orientations  // 스캔의 시작과 끝의 방향(orientation)을 확인한다.
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;

  _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());  // 포인트클라우드를 사용하는 라이다의 layer갯수에 맞게 resize해줌.

  // clear all scanline points    // 결과를 집어넣을 포인트 클라우드의 모든 (scan line의) 포인트들을 clear해 준다.
  std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto&&v) {v.clear(); }); 

  // extract valid points from input cloud.   // 입력된 클라우드로 부터 유효한 포인트들을 추출하는 for문.
  _laserCloudSur.clear();



  //pcl::PointXYZI point;
  ////////////////////////////////////////
  pcl::PointXYZRGBNormal point;

  int xp, yp, Idx;

  float depthL, xl, yl, zl;
  cv::Mat_<float> xyz_C(3,1);
  cv::Mat_<float> xyz_L(4,1);
  ////////////////////////////////////////


  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    point.normal_x = -1;
    point.normal_y = -1;
    point.normal_z = -1;

    // skip NaN and INF valued points.     // NaN 이나 INF인 포인트들은 건너뛴다.
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points             // 좌표 값이 0인 포인트들은 건너뛴다.
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID.                                       // 포인트의 세로 각과 어떤 layer에서 스캔되었는지 계산한다.
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z)); // 세로각.
    int scanID = _scanMapper.getRingForAngle(angle);                                     // 어떤 layer에서 스캔되었는지.
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){                    // 만일 layer가 사용하는 라이다의 layer갯수 이상이거나 0보다 작다면 건너뛴다.
      continue;
    }

    // calculate horizontal point angle.        // 포인트의 가로 각을 계산한다.
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {                          // 스캔의 절반(180도)를 지나지 않은경우
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {                                    // 스캔의 절반(180도)을 지난 경우
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation.   // 포인트의 방향(orientation)을 기반으로 relative scan time을 계산한다.
    float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);
    point.curvature = scanID + relTime;


    if(point.z > 0)
    {
      xyz_L << point.x, point.y, point.z, 1; // 라이다 좌표.
      xyz_C = KE * xyz_L;  // 행렬을 곱하여 라이다 좌표를 카메라 좌표로 변환.

      xp = round(xyz_C[0][0]/xyz_C[2][0]);  // 변환한 x, y 좌표. s를 나눠주어야 함.
      yp = round(xyz_C[1][0]/xyz_C[2][0]);

      if(0 <= xp && xp < 1280)  // 1280,720 이내의 픽셀 좌표를 가지는 값들에 대해서만 depth값을 추가로 비교.
      {
        if(0 <= yp && yp < 720) // 추가 픽셀들이 5x5인 경우 2 1278  321, 960
        {
          Idx = xp + 1280*yp;   // 이미지의 각 픽셀에 해당하는 depth값을 얻기 위한 index.

          if(std::isfinite(depths[Idx])){
              point.normal_x = xp;
              point.normal_y = yp;
              point.normal_z = depths[Idx];
          }

        }

      }

    }
    ///////////////// ///////////////////////////// ////////////////////
    
    projectPointToStartOfSweep(point, relTime);   // parameter로 입력한 포인트를 sweep의 시작지점으로 투영해주는 메소드. 
                                                  // 매 포인트를 이렇게 처리해주어야, 한번의 sweep에 대해 하나의 제대로 된 포인트 클라우드를 생성할 수 있다.
                                                  // sweep이 
                                                
    _laserCloudScans[scanID].push_back(point);    // 위와 같이 처리(process)한 포인트를 포인트 클라우드의 해당 layer 부분에 저장.
  }

  processScanlines(scanTime, _laserCloudScans);   // 위와 같은 process를 통해 생성된 포인트 클라우드를 기반으로 _laserCloud, 
                                                // _cornerPointsSharp, _cornerPointsLessSharp, _surfacePointsFlat, _surfacePointsLessFlat
                                                // 그리고 _imuTrans를 생성한다.

  publishResult();    // 다른 node로 그것들을 publish한다.
}

} // end namespace loam