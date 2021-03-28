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

#define lidarPoint_Thick 0
#define camToLidar 0

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
  _subLeftRectified  = node.subscribe("/zed2/zed_node/left/image_rect_color", 10, &MultiScanRegistration::imageLeftRectifiedHandler, this);
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

/////////////////////////////////////////////////////
void MultiScanRegistration::imageLeftRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg) {
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

void MultiScanRegistration::imageRightRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("Right Rectified image received from ZED - Size: %dx%d",
    //          msg->width, msg->height);
    
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }

    _mat_right = cv_ptr->image;
}

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

    // int idx = 0;
    // for(int y = 0; y < 720; y++){
    //   for(int x = 0; x < 1280; x++){
    //     std::cout << "x:" << x << " y:" << y << std::endl;
    //     std::cout << "image:" << _mat_depth.at<float>(y, x) << " array:" << depths[idx] << std::endl;
    //     idx++;
    //   }
    // }
    

    // // Image coordinates of the center pixel
    // int u = msg->width / 2;
    // int v = msg->height / 2;

    // // Linear index of the center pixel
    // int centerIdx = u + msg->width * v;
    // ROS_INFO("Center distance : %g m", depths[centerIdx]);

    // // Output the measure
    // for(int j = 0; j < 1280; j++){
    //    ROS_INFO(" distance : %g m", depths[j]);
    // }
    
}

void MultiScanRegistration::leftcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg) {

    // cv::Mat 형태로 저장.
    // K = (cv::Mat_<float>(3,3) <<  msg->K[0], msg->K[1], msg->K[2],
    //                               msg->K[3], msg->K[4], msg->K[5],
    //                               msg->K[6], msg->K[7], msg->K[8] );

    K = (cv::Mat_<float>(3,3) <<  msg->P[0], msg->P[1], msg->P[2],
                                  msg->P[4], msg->P[5], msg->P[6],
                                  msg->P[8], msg->P[9], msg->P[10] );

    // std::cout << " fx : " << K.at<float>(0) << " fy : " << K.at<float>(4) << " cx : " << K.at<float>(2) << " cy : " << K.at<float>(5) << std::endl;
    

    // 두 행렬 곱하기.
    KE = K * E;

#if camToLidar
    // pseudo 역행렬 만들기.
    cv::Mat KE_t = KE.t();
    cv::Mat tmp = KE_t * KE;
    cv::Mat tmp_inv = tmp.inv();
    pseu_inv_KE = tmp_inv * KE_t;
#endif    

    // std::cout << "IN leftcamInfoHandler!!!!!!!!" << std::endl;

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

/////////////////// 중복 제거 알고리즘 ///////////////////
bool MultiScanRegistration::isOverlap(const pcl::PointXYZRGBNormal& point){
  
  // 중복 제거 알고리즘 용.
  
  int preci = 100;  // cm 범위에 대해 반올림.
  float tmpArr[3];
  std::string tmpString;
  
  // 1. cm 범위에 대해 반올림. (100을 곱한 뒤, round()후 다시 100을 나눈다.)
  tmpArr[0] = round(point.x * preci) / preci;
  tmpArr[1] = round(point.y * preci) / preci;
  tmpArr[2] = round(point.z * preci) / preci;

  // 2. 각 좌표를 string으로 바꾼 후, 하나의 string으로 합침.
  tmpString = std::to_string(tmpArr[0]) + ',' + std::to_string(tmpArr[1]) + ',' + std::to_string(tmpArr[2]);

  // 3. 중복된 좌표인지 체크
  iter = overlapCheck.find(tmpString);

  // 4. 만일 존재하는 경우 continue
  if(iter != overlapCheck.end())
    return true;

  // 5. 존재하지 않는경우 set에 추가.
  overlapCheck.insert(tmpString);

  return false;
}

// ////////////// 임의의 픽셀 포인트 추가 알고리즘 //////////////
// void MultiScanRegistration::makePixelPoint(const pcl::PointXYZRGBNormal& point, int scanID){
//   // 포인트 추가 알고리즘 용.
//   float lineEq[3];
//   float tmpVec1[3];
//   float tmpVec2[3];
//   float normalVec[3];
//   float centroid[3];
//   float v1[3];
//   float v2[3];
//   float sizev1, sizev2;
//   float dist;
//   float r, maxRadius;
//   float t;
//   float inc;
//   int thresh;

//   int xp, yp, Idx, B, G, R;
//   float depthL, xl, yl, zl;
//   uchar* p;
//   int channels = _mat_left.channels();
//   cv::Mat_<float> xyz_L(4,1);
//   cv::Mat_<float> xyz_C(3,1);
//   std::uint32_t rgb;

//   pcl::PointXYZRGB pixPoint;

//   //std::cout << "in the makePixelPoint function!" << '\n';

//   // 1. 같은 layer의 이전 point들 체크
//   if(prevPointAt[scanID].size() < 2){
//     prevPointAt[scanID].push(point);  // 새 점을 포함하는 점이 2개 이하인 경우 저장하고 넘어감.
//   }
//   // 3개 이상인 경우
//   else{   
//     // 1. 3개의 점들이 서로 직선상에 존재하는지 확인한다.
//     // D와 d1으로 직선을 만들어, d2가 그 위에 존재하는지 확인한다.
//     tmpVec1[0] = prevPointAt[scanID].front().x - point.x;
//     tmpVec1[1] = prevPointAt[scanID].front().y - point.y;
//     tmpVec1[2] = prevPointAt[scanID].front().z - point.z;

//     lineEq[0] = (prevPointAt[scanID].back().x - point.x)/tmpVec1[0];
//     lineEq[1] = (prevPointAt[scanID].back().y - point.y)/tmpVec1[1];
//     lineEq[2] = (prevPointAt[scanID].back().z - point.z)/tmpVec1[2];

//     // 세 점이 직선상에 존재하는 경우 queue에 새 점을 추가하고 넘어간다.
//     if(lineEq[0] == lineEq[1] && lineEq[0] == lineEq[2]){   
//       prevPointAt[scanID].pop();
//       prevPointAt[scanID].push(point);
//     }
//     // 직선상에 존재하지 않는 경우.
//     else
//     {
//       // 2. 3개의 점으로 만들어지는 평면 위에 존재하는 서로 직교하는 두개의 벡터 v1과 v2를 구한다.
//       // 평면의 법선벡터 V를 구한다.
//       tmpVec2[0] = lineEq[0] * tmpVec1[0];
//       tmpVec2[1] = lineEq[1] * tmpVec1[1];
//       tmpVec2[2] = lineEq[2] * tmpVec1[2];

//       // V는 tmpVec1 과 tmpVec2의 외적이다.
//       normalVec[0] = tmpVec2[1]*tmpVec1[2] - tmpVec2[2]*tmpVec1[1];
//       normalVec[1] = tmpVec2[2]*tmpVec1[0] - tmpVec2[0]*tmpVec1[2];
//       normalVec[2] = tmpVec2[0]*tmpVec1[1] - tmpVec2[1]*tmpVec1[0];

//       // 3개의 점으로 만들어지는 삼각형의 무게중심을 구한다.
//       centroid[0] = (prevPointAt[scanID].front().x + prevPointAt[scanID].back().x + point.x) / 3;
//       centroid[1] = (prevPointAt[scanID].front().y + prevPointAt[scanID].back().y + point.y) / 3;
//       centroid[2] = (prevPointAt[scanID].front().z + prevPointAt[scanID].back().z + point.z) / 3;

//       // 무게중심 - p3로 벡터 v1을 구한다.
//       v1[0] = centroid[0] - point.x;
//       v1[1] = centroid[1] - point.y;
//       v1[2] = centroid[2] - point.z;

//       // v1과 V에 수직인 벡터 v2를 구한다. (v1 x V) (두 벡터의 외적)
//       v2[0] = v1[1]*normalVec[2] - v1[2]*normalVec[1];
//       v2[1] = v1[2]*normalVec[0] - v1[0]*normalVec[2];
//       v2[2] = v1[0]*normalVec[1] - v1[1]*normalVec[0];

//       // v1, v2 모두 unit vector로 바꾼다.
//       sizev1 = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
//       sizev2 = sqrt(v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]);

//       v1[0] /= sizev1;
//       v1[1] /= sizev1;
//       v1[2] /= sizev1;

//       v2[0] /= sizev2;
//       v2[1] /= sizev2;
//       v2[2] /= sizev2;

//       // 3. 평면 위에 존재하는 중심이 p3이고, 반지름이 r인 원의 원주상에 존재하는 점들의 좌표를 구한다.

//       dist  = 0.001;
//       maxRadius = 0.003;
      
//       for(float r = dist; r <= maxRadius; r += dist){
//         thresh = (2*M_PI*r/dist);
//         inc = 2 * asin(dist/2*r) * M_PI/180;
//         t = 0;

//         //std::cout << "in the first for loop!" << '\n';

//         for(int k = 0; k < thresh; k++){
//           // 해당 좌표로 포인트를 생성한다.
//           pixPoint.x = point.x + (r*cos(t)) * v1[0] + (r*sin(t)) * v2[0];
//           pixPoint.y = point.y + (r*cos(t)) * v1[1] + (r*sin(t)) * v2[1];
//           pixPoint.z = point.z + (r*cos(t)) * v1[2] + (r*sin(t)) * v2[2];

//           // skip NaN and INF valued points.     // NaN 이나 INF인 포인트들은 건너뛴다.
//           if (!pcl_isfinite(pixPoint.x) ||
//               !pcl_isfinite(pixPoint.y) ||
//               !pcl_isfinite(pixPoint.z)) {
//             continue;
//           }
//           // skip zero valued points             // 좌표 값이 0인 포인트들은 건너뛴다.
//           if (pixPoint.x * pixPoint.x + pixPoint.y * pixPoint.y + pixPoint.z * pixPoint.z < 0.0001) {
//             continue;
//           }

//           //std::cout << "x,y,z: " << pixPoint.x << " " << pixPoint.y << " " << pixPoint.z << '\n';

//           if(isOverlap(pixPoint)){
//             t += inc;
//             continue;
//           }

//           // 행렬을 이용해 이미지로 투영한다.
//           xyz_L << pixPoint.x, pixPoint.y, pixPoint.z, 1; // 라이다 좌표.
//           xyz_C = KE * xyz_L;  // 행렬을 곱하여 라이다 좌표를 카메라 좌표로 변환.

//           xp = round(xyz_C[0][0]/xyz_C[2][0]);  // 변환한 x, y, z 좌표. s를 나눠주어야 함.
//           yp = round(xyz_C[1][0]/xyz_C[2][0]);

//           if(0 <= xp && xp < 1280)  // 1280,720 이내의 픽셀 좌표를 가지는 값들에 대해서만 depth값을 추가로 비교.
//           {
//             if(0 <= yp && yp < 720)
//             {
//               //std::cout << "xp,yp: " << xp << " " << yp << '\n';

//               // 5. depth가 비슷할 경우 색을 입힌다.
//               p = _mat_left.ptr<uchar>(yp);
//               B = p[xp*channels + 0];   // left 이미지에서 컬러값 추출.
//               G = p[xp*channels + 1];
//               R = p[xp*channels + 2]; 

//               xl = pixPoint.x - 0.165; // 라이다 점의 depth값을 구할 때, 하드웨어의 위치관계를 고려해 주어야 한다.
//               yl = pixPoint.y + 0.066;
//               zl = pixPoint.z - 0.0444;

//               depthL = sqrt(xl*xl + yl*yl + zl*zl);

//               Idx = xp + 1280*yp;

//               if(std::isfinite(depths[Idx])){
                
//                 if(((depthL-0.2) < depths[Idx]) && (depths[Idx] < (depthL+0.2))){
//                   rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
//                   pixPoint.rgb = *reinterpret_cast<float*>(&rgb);

//                   // 6. 점을 pixelCloud에 입력한다.

//                   //std::cout << "Adding point!" << '\n';
//                   pixelCloud().push_back(pixPoint); 
//                 }
//               }
//             }
//           }  

//           t += inc;
//         }
//       }
//       // 7. prevPointAt[scanID] 에 첫 점을 내보내고 새 점 p3를 저장한다.
//       prevPointAt[scanID].pop();
//       prevPointAt[scanID].push(point);
//     }
    
//   }
// }

//////////// 포인트의 normal vector를 계산해 부여하는 알고리즘 ////////////
void MultiScanRegistration::calcNorVec(pcl::PointXYZRGBNormal& point, int scanID){

  // 포인트 추가 알고리즘 용.
  float lineEq[3];
  float tmpVec1[3];
  float tmpVec2[3];

  // 1. 같은 layer의 이전 point들 체크
  if(prevPointAt[scanID].size() < 2){
    prevPointAt[scanID].push(point);  // 새 점을 포함하는 점이 2개 이하인 경우 저장하고 넘어감.

    point.normal_x = -1;
    point.normal_y = -1;
    point.normal_z = -1;
  }
  // 3개 이상인 경우
  else{   
    // 1. 3개의 점들이 서로 직선상에 존재하는지 확인한다.
    // D와 d1으로 직선을 만들어, d2가 그 위에 존재하는지 확인한다.
    tmpVec1[0] = prevPointAt[scanID].front().x - point.x;
    tmpVec1[1] = prevPointAt[scanID].front().y - point.y;
    tmpVec1[2] = prevPointAt[scanID].front().z - point.z;

    lineEq[0] = (prevPointAt[scanID].back().x - point.x)/tmpVec1[0];  // 3개의 점들 중 나머지 하나의 점을 직선에 방정식에 넣은것.
    lineEq[1] = (prevPointAt[scanID].back().y - point.y)/tmpVec1[1];
    lineEq[2] = (prevPointAt[scanID].back().z - point.z)/tmpVec1[2];

    // 세 점이 직선상에 존재하는 경우 queue에 새 점을 추가하고 넘어간다.
    if(lineEq[0] == lineEq[1] && lineEq[0] == lineEq[2]){   
      prevPointAt[scanID].pop();
      prevPointAt[scanID].push(point);

      point.normal_x = -1;
      point.normal_y = -1;
      point.normal_z = -1;
    }
    // 직선상에 존재하지 않는 경우.
    else{
      // 평면의 법선벡터 V를 구한다.
      tmpVec2[0] = lineEq[0] * tmpVec1[0];
      tmpVec2[1] = lineEq[1] * tmpVec1[1];
      tmpVec2[2] = lineEq[2] * tmpVec1[2];

      // V는 tmpVec1 과 tmpVec2의 외적이다.
      point.normal_x = tmpVec2[1]*tmpVec1[2] - tmpVec2[2]*tmpVec1[1];
      point.normal_y = tmpVec2[2]*tmpVec1[0] - tmpVec2[0]*tmpVec1[2];
      point.normal_z = tmpVec2[0]*tmpVec1[1] - tmpVec2[1]*tmpVec1[0];

    }
  }
}

////////////////////////////////////////////////////////////////////


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

  int xp, yp, Idx, B, G, R;
  
  uchar* p;
  int channels = _mat_left.channels();

  float depthL, xl, yl, zl;
  cv::Mat_<float> xyz_C(3,1);
  cv::Mat_<float> xyz_L(4,1);
  
  int zedepth, lidepth;
  float zedepthf, lidepthf;
  std::uint32_t rgb;


  // cam to lidar용.
  cv::Mat_<float> xy_C(3,1);
  ////////////////////////////////////////

  
  ////////////////////////////////////////
  double fx_d = K.at<float>(0,0);
  double fy_d = K.at<float>(1,1);
  double cx_d = K.at<float>(0,2);
  double cy_d = K.at<float>(1,2);

  pcl::PointXYZRGBNormal pixpoint;
  int pixRange = 0; // 1 = 3x3, 2 = 5x5, 3 = 7x7 
  ////////////////////////////////////////


  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    point.normal_x = 0;
    point.normal_y = 0;
    point.normal_z = 0;

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

      if(pixRange <= xp && xp < 1280-pixRange)  // 1280,720 이내의 픽셀 좌표를 가지는 값들에 대해서만 depth값을 추가로 비교.
      {
        if(pixRange <= yp && yp < 720-pixRange) // 추가 픽셀들이 5x5인 경우 2 1278  321, 960
        {
          // std::cout << "Entering makePixelPoint!" << '\n';
          // calcNorVec(point, scanID);
          // std::cout << "point의 n_x, n_y, n_z: " << point.normal_x << " " << point.normal_y << " " << point.normal_z <<'\n';

          p = _mat_left.ptr<uchar>(yp);
          B = p[xp*channels + 0];   // left 이미지에서 컬러값 추출.
          G = p[xp*channels + 1];
          R = p[xp*channels + 2]; 

          xl = point.x - 0.165; // 라이다 점의 depth값을 구할 때, 하드웨어의 위치관계를 고려해 주어야 한다.
          yl = point.y + 0.066;
          zl = point.z - 0.0444;

          depthL = sqrt(xl*xl + yl*yl + zl*zl);

          Idx = xp + 1280*yp;   // 이미지의 각 픽셀에 해당하는 depth값을 얻기 위한 index.

          if(std::isfinite(depths[Idx])){

            zedepthf = depths[Idx];
            lidepthf = depthL;// + 0.0444; // depth에 대한 translation

            //std::cout << "scanID: " << scanID << std::endl;
            
            if(((lidepthf-0.2) < zedepthf) && (zedepthf < (lidepthf+0.2))){

              // // 만일 동일한 점이 존재하는 경우 다음 point로 넘어간다.
              // if(isOverlap(point))
              //   continue;
              // point.x = -((xp - cx_d) * depthL / fx_d);
              // point.y = -((yp - cy_d) * depthL / fy_d);
              // point.z = depthL; //depths[Idx];

              rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
              point.rgb = *reinterpret_cast<float*>(&rgb);

              point.normal_x = 1;
              point.normal_y = 1;
              point.normal_z = 1;


              // //this made by INHYEOK, AVERAGEING FILTER
              // float zed_dep_sum = 0;
              // float zed_dep_avg = 0;
              // int array_num = 0;
              // for(int x =  xp-pixRange; x < xp+pixRange; x++){
              //   for(int y = yp-pixRange; y < yp+pixRange; y++){
              //     int i = x + 1280 * y;
              //     zed_dep_sum += depths[i];
              //     array_num ++;
              //   }
              // }
              // zed_dep_avg = (zed_dep_sum / array_num);
              // if(((lidepthf-0.2) < zed_dep_avg) && (zed_dep_avg < (lidepthf+0.2)))
              // {
              //   for(int x = xp-pixRange; x < xp+pixRange; x++){
              //     for(int y = yp-pixRange; y < yp+pixRange; y++){

              //       pixpoint.x = -((x - cx_d) * depthL / fx_d);
              //       pixpoint.y = -((y - cy_d) * depthL / fy_d);
              //       pixpoint.z = depthL;

              //       p = _mat_left.ptr<uchar>(y);
              //       B = p[x*channels + 0];   // left 이미지에서 컬러값 추출.
              //       G = p[x*channels + 1];
              //       R = p[x*channels + 2]; 

              //       rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
              //       pixpoint.rgb = *reinterpret_cast<float*>(&rgb);

              //       pixpoint.normal_x = 1;
              //       pixpoint.normal_y = 1;
              //       pixpoint.normal_z = 1;

              //       _laserCloudSur.push_back(pixpoint);
              //     }
              //   }
              // }
              

              // //original
              // for(int x = xp-pixRange; x < xp+pixRange; x++){
              //   for(int y = yp-pixRange; y < yp+pixRange; y++){

              //     pixpoint.x = -((x - cx_d) * depthL / fx_d);
              //     pixpoint.y = -((y - cy_d) * depthL / fy_d);
              //     pixpoint.z = depthL;

              //     p = _mat_left.ptr<uchar>(y);
              //     B = p[x*channels + 0];   // left 이미지에서 컬러값 추출.
              //     G = p[x*channels + 1];
              //     R = p[x*channels + 2]; 

              //     rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
              //     pixpoint.rgb = *reinterpret_cast<float*>(&rgb);

              //     pixpoint.normal_x = 1;
              //     pixpoint.normal_y = 1;
              //     pixpoint.normal_z = 1;

              //     // _laserCloudSur.push_back(pixpoint);
              //     laserCloud().push_back(pixpoint);
              //   }
              // }
              


            }

          }

        }

      }

    }

    if(point.normal_x == 0 && point.normal_y == 0 && point.normal_z == 0){// && point.normal_y != 1 && point.normal_x != 1){          // 컬러가 없는 포인트들엔 검정을 할당.
      rgb = (static_cast<std::uint32_t>(0) << 16 | static_cast<std::uint32_t>(0) << 8 | static_cast<std::uint32_t>(0));
      point.rgb = *reinterpret_cast<float*>(&rgb);
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




#if camToLidar
  /////// 카메라 -> 라이다: 띄우기 /////////////////
  pcl::PointXYZRGB point2;
  // pcl::PointXYZRGB prevPoint2;
  // float dist = 0;

  for(int x = 0, y = 0, j = 0; x < 1280, y < 720; x++, j++)
  {
    xy_C << x, y, 1; // 카메라 좌표(픽셀)
    xyz_L = pseu_inv_KE * xy_C;  // 역행렬을 곱하여 카메라 좌표를 라이다 좌표로 변환.

    //std::cout << "xyz_L[3][0] : " << xyz_L[3][0] << std::endl;
    point2.z = xyz_L[2][0];///xyz_L[3][0];//(depths[x + 1280*y])*0.01; // 0.25;
    point2.x = xyz_L[0][0];///xyz_L[3][0]; //* 0.1236873778;    // 16:9로 맞춰주기 위한 비례 상수. 
    point2.y = xyz_L[1][0];///xyz_L[3][0];
    // for (int i = 0; i < cloudSize; i++) {
    //   if((int(point2.x*1000) == (int)(laserCloudIn[i].y*1000) && point2.y == laserCloudIn[i].z){
    //     point2.z = laserCloudIn[i].x;
    //   }
    // }
    
    //depths[j];

    // skip NaN and INF valued points.     // NaN 이나 INF인 포인트들은 건너뛴다.
    if (!pcl_isfinite(point2.x) ||
        !pcl_isfinite(point2.y) ||
        !pcl_isfinite(point2.z)) {
      continue;
    }

    // skip zero valued points             // 좌표 값이 0인 포인트들은 건너뛴다.
    if (point2.x * point2.x + point2.y * point2.y + point2.z * point2.z < 0.0001) {
      continue;
    }

    //std::cout << "xyzl: " << point2.x << " " << point2.y << " " << point2.z << " " << std::endl;

    p = _mat_left.ptr<uchar>(y);
    B = p[x*channels + 0];   // left 이미지에서 컬러값 추출.
    G = p[x*channels + 1];
    R = p[x*channels + 2];

    ////////////색상 RGB 적용//////////////
    std::uint32_t rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
    point2.rgb = *reinterpret_cast<float*>(&rgb);
    /////////////////////////////////////////

    // if(j != 0){
    //   dist = sqrt( pow((point2.x - prevPoint2.x),2) + pow((point2.y - prevPoint2.y),2) + pow((point2.z - prevPoint2.z),2) );
    //   // std::cout << "current pixel coordinate: (" << x  << ", " << y << ")"<< '\n';
    //   // std::cout << "distance between two pixel points: " << dist << '\n';
    // }

    pixelCloud().push_back(point2); 

    // prevPoint2 = point2;

    if(x >= 1279){
      x = 0;
      y = y + 1;
    }
  }
#endif





// // 라이다 depth이미지 띄우기
          // _mat_lidar_depth.at<float>(yp, xp) = depthL;
          // _mat_lidar_depth.at<cv::Vec3b>(yp, xp)[0] = B;
          // _mat_lidar_depth.at<cv::Vec3b>(yp, xp)[1] = G;
          // _mat_lidar_depth.at<cv::Vec3b>(yp, xp)[2] = R;
                                                  // 같은 process를 통해 생성된 포인트 클라우드를 기반으로 _laserCloud, 
                                                  // _cornerPointsSharp, _cornerPointsLessSharp, _surfacePointsFlat, _surfacePointsLessFlat
                                                  // 그리고 _imuTrans를 생성한다.


          // // 라이다 depth이미지 띄우기
          // _mat_lidar_depth.at<float>(yp, xp) = depthL;
          // _mat_lidar_depth.at<cv::Vec3b>(yp, xp)[0] = B;
          // _mat_lidar_depth.at<cv::Vec3b>(yp, xp)[1] = G;
          // _mat_lidar_depth.at<cv::Vec3b>(yp, xp)[2] = R;

// ////////////////////////////////
// int orderCheckR = 255;
// int orderCheckG = 0;
// int orderCheckB = 0;
// ////////////////////////////////

// rgb = (static_cast<std::uint32_t>(orderCheckR) << 16 | static_cast<std::uint32_t>(orderCheckG) << 8 | static_cast<std::uint32_t>(orderCheckB));
// if(orderCheckG == 0 && orderCheckB == 0){   // 100 -> 000
//   orderCheckR--;
//   if(orderCheckR == 1){   // -> 010
//     orderCheckR = 0;
//     orderCheckG = 255;
//     orderCheckB = 0;
//   }
// }else if(orderCheckR == 0 && orderCheckB == 0){ // 010 -> 000
//   orderCheckG--;
//   if(orderCheckG == 1){   // -> 001
//     orderCheckR = 0;
//     orderCheckG = 0;
//     orderCheckB = 255;
//   }
// }else if(orderCheckR == 0 && orderCheckG == 0){ // 001 -> 000
//   orderCheckB--;
//   if(orderCheckB == 1){ // -> 110
//     orderCheckR = 255;
//     orderCheckG = 255;
//     orderCheckB = 0;
//   }
// }else if(orderCheckG == 255 && orderCheckB == 0){ // 110 -> 010
//   orderCheckR--;
//   if(orderCheckB == 1){ // -> 011
//     orderCheckR = 0;
//     orderCheckG = 255;
//     orderCheckB = 255;
//   }
// }else if(orderCheckR == 0 && orderCheckB == 255){ // 011 -> 001
//   orderCheckG--;
//   if(orderCheckG == 1){ // -> 101
//     orderCheckR = 255;
//     orderCheckG = 0;
//     orderCheckB = 255;
//   }
// }else if(orderCheckR == 255 && orderCheckG == 0){ // 101 -> 100
//   orderCheckB--;
//   if(orderCheckB == 1){ // -> 100
//     orderCheckR = 255;
//     orderCheckG = 0;
//     orderCheckB = 0;
//   }
// }

#if lidarPoint_Thick
              ///////////// 주위 픽셀들 추가 ///////////////
              pcl::PointXYZRGBNormal point2;
              int radius = 3;
              float lirad = 0.001;
              // float xsur = (point.x-lirad);
              float ysur = (point.y-lirad);

              if(xp > (radius-1) && xp < (1280-radius) && yp > (radius-1) && yp < (720-radius))
              {
                // for(int x = (xp-radius); x < xp + (radius+1); x++)
                // {
                  for(int y = (yp-radius); y < yp + (radius+1); y++)
                  {
                    if(y == yp)//x == xp && 
                    {
                      continue;
                    }

                    point2.x = point.x; //xsur;
                    point2.y = ysur;
                    point2.z = point.z;


                    p = _mat_left.ptr<uchar>(y);
                    B = p[xp*channels + 0];   // left 이미지에서 컬러값 추출.
                    G = p[xp*channels + 1];
                    R = p[xp*channels + 2];

                    rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
                    point2.rgb = *reinterpret_cast<float*>(&rgb);

                    point2.normal_x = 1;
                    point2.normal_y = 1;
                    point2.normal_z = 1;


                    projectPointToStartOfSweep(point2, relTime);
                    _laserCloudSur.push_back(point2);
                    
                    ysur += (lirad/radius);
                  }
                //   xsur += lirad / radius;
                // }
              }
              ///////////////////////////////////////////
#endif     

#if lidarPoint_Thick
  ///////////// 주위 픽셀들 추가 ///////////////
  laserCloud() += _laserCloudSur;
  ///////////////////////////////////////////
#endif



// pcl::PointXYZRGBNormal pointRGB;
//   uchar* p;
//    int channels = _mat_left.channels();
//    int Idx, B, G, R;
//     std::uint32_t rgb;

//    double fx_d = K.at<float>(0,0);
//    double fy_d = K.at<float>(1,1);
//    double cx_d = K.at<float>(0,2);
//    double cy_d = K.at<float>(1,2);

//    for(int x = 321; x <= 960; x++) //0부터 1280
//     {
//       for(int y = 0; y <= 720; y++)
//       {
//          Idx = x + 1280*y;

//          if(std::isfinite(depths[Idx])){
//             if(depths[Idx] > 0)
//             {
//                pointRGB.x = -((x - cx_d) * depths[Idx] / fx_d);
//                pointRGB.y = -((y - cy_d) * depths[Idx] / fy_d);
//                pointRGB.z = depths[Idx];

//                if(!isOverlap(pointRGB)){
//                   p = _mat_left.ptr<uchar>(y);
//                   B = p[x*channels + 0];   // left 이미지에서 컬러값 추출.
//                   G = p[x*channels + 1];
//                   R = p[x*channels + 2]; 

//                   rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
//                   pointRGB.rgb = *reinterpret_cast<float*>(&rgb);

//                   _laserCloudSur.push_back(pointRGB);
                  
//                }
//             }
            
//          }

         
         
//       }
//    }
  
//   laserCloud() += _laserCloudSur;