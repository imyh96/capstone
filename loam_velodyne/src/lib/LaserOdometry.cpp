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
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014

#include <pcl/filters/filter.h>

#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/common.h"
#include "math_utils.h"

namespace loam
{

  using std::sin;
  using std::cos;
  using std::asin;
  using std::atan2;
  using std::sqrt;
  using std::fabs;
  using std::pow;


  LaserOdometry::LaserOdometry(float scanPeriod, uint16_t ioRatio, size_t maxIterations):
    BasicLaserOdometry(scanPeriod, maxIterations),
    _ioRatio(ioRatio)
  {
    // initialize odometry and odometry tf messages
    _laserOdometryMsg.header.frame_id = "/camera_init";
    _laserOdometryMsg.child_frame_id  = "/laser_odom";

    _laserOdometryTrans.frame_id_       = "/camera_init";
    _laserOdometryTrans.child_frame_id_ = "/laser_odom";
  }


  bool LaserOdometry::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
  { // 초기에 변수를 node, private node 로 설정
    // fetch laser odometry params
    float fParam;
    int iParam;

    if (privateNode.getParam("scanPeriod", fParam)) // 스캔주기(scan period)를 입력받음
    {
      if (fParam <= 0) //스캔주기가 음수또는 0이면 error발생
      {
        ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
        return false;
      }
      else // 스캔주기가 정상이면,
      {
        setScanPeriod(fParam); // 스캔주기 입력
        ROS_INFO("Set scanPeriod: %g", fParam);
      }
    }

    if (privateNode.getParam("ioRatio", iParam)) // 입출력 비(io ratio) 도 동일하게 설정.
    {
      if (iParam < 1)
      {
        ROS_ERROR("Invalid ioRatio parameter: %d (expected > 0)", iParam);
        return false;
      }
      else
      {
        _ioRatio = iParam;
        ROS_INFO("Set ioRatio: %d", iParam);
      }
    }

    if (privateNode.getParam("maxIterations", iParam)) // 최대 itreation횟수도 설정.
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

    if (privateNode.getParam("deltaTAbort", fParam)) // 델타T abort.
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

    if (privateNode.getParam("deltaRAbort", fParam)) // 델타R abort.
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
 
    // advertise laser odometry topics   // 다른 node로 보낼 값들 4가지를 advertise.
    _pubLaserCloudCornerLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2); // 이 node에서 처리된 후의 corner 특징점 클라우드.
    _pubLaserCloudSurfLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);     // 이 node에서 처리된 후의 surface 특징점 클라우드.
    _pubLaserCloudFullRes = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);           // 이 node에서 처리된 후의 full resolution 포인트 클라우드.
    _pubLaserOdometry = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);                   // 이 node에서 연산된 odometry.

    // _pubPixelCloud            = node.advertise<sensor_msgs::PointCloud2>("/pixel_cloud", 2);

    // subscribe to scan registration topics  // scan registraion 노드로 부터 6가지 메세지를 subscribe. 바로 각각의 핸들러 메소드로 보낸다.
    _subCornerPointsSharp = node.subscribe<sensor_msgs::PointCloud2>  // 모서리 edge
      ("/laser_cloud_sharp", 2, &LaserOdometry::laserCloudSharpHandler, this);

    _subCornerPointsLessSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_sharp", 2, &LaserOdometry::laserCloudLessSharpHandler, this);

    _subSurfPointsFlat = node.subscribe<sensor_msgs::PointCloud2>  // 표면 planar
      ("/laser_cloud_flat", 2, &LaserOdometry::laserCloudFlatHandler, this);

    _subSurfPointsLessFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_flat", 2, &LaserOdometry::laserCloudLessFlatHandler, this);

    _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2> // clould full resolution 포인트 클라우드
      ("/velodyne_cloud_2", 2, &LaserOdometry::laserCloudFullResHandler, this);

    _subImuTrans = node.subscribe<sensor_msgs::PointCloud2>  // IMU로 부터의 transformation (변환행렬).
      ("/imu_trans", 5, &LaserOdometry::imuTransHandler, this);


    return true;
  }
  //setup 끝


  void LaserOdometry::reset() // 리셋
  {
    _newCornerPointsSharp = false;
    _newCornerPointsLessSharp = false;
    _newSurfPointsFlat = false;
    _newSurfPointsLessFlat = false;
    _newLaserCloudFullRes = false;
    _newImuTrans = false;



  }


  //////////////  Handler 함수는 오류를 잡아주는 과정으로 모두 같은 과정///////////////

  void LaserOdometry::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg)  // subscribe해 온 메세지.
  {
    _timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp; // subscribe해 온 포인트 클라우드에 저장되어 있던 스캔된 시간을 저장.

    cornerPointsSharp()->clear(); // 입력하기 전, _cornerPointsSharp변수에 남은 이전 데이터를 정리함.

    // auto const& cornerpointsSharp()
    // auto => 자료형 알아서 선택해줌  여기선 pc일듯, const& 값을 변환시키지 않도록하고 참조(reference)

    pcl::fromROSMsg(*cornerPointsSharpMsg, *cornerPointsSharp()); // subscribe해 온 msg를 _cornerPointsSharp에 저장

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsSharp(), *cornerPointsSharp(), indices);  // 포인트 클라우드로 부터 NaN값을 가지는 포인트들은 제거해 준다.
                                                                                        // NaN : not a number => 컴퓨터가 표시하지 못하는 숫자들
                                                                                        // 0/0 , 문자가 들어간 데이터, 허수등 
  

    _newCornerPointsSharp = true; // 새로운 데이터가 입력 되었음을 나타내는 flag를 ON.
  }

  void LaserOdometry::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg)  // subscribe해 온 msg를 _cornerPointsLessSharp에 저장
  {
    _timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp;

    cornerPointsLessSharp()->clear();
    pcl::fromROSMsg(*cornerPointsLessSharpMsg, *cornerPointsLessSharp());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsLessSharp(), *cornerPointsLessSharp(), indices); // less sharp corner
    _newCornerPointsLessSharp = true;
  }

  void LaserOdometry::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg)  // subscribe해 온 msg를 _surfPointsFlat에 저장
  {
    _timeSurfPointsFlat = surfPointsFlatMsg->header.stamp;

    surfPointsFlat()->clear();
    pcl::fromROSMsg(*surfPointsFlatMsg, *surfPointsFlat());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsFlat(), *surfPointsFlat(), indices); // flat한 표면
    _newSurfPointsFlat = true;
  }

  void LaserOdometry::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg)  // subscribe해 온 msg를 _surfPointsLessFlat에 저장
  {
    _timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp;

    surfPointsLessFlat()->clear();
    pcl::fromROSMsg(*surfPointsLessFlatMsg, *surfPointsLessFlat());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsLessFlat(), *surfPointsLessFlat(), indices); //less flat surface
    _newSurfPointsLessFlat = true;
  }

  void LaserOdometry::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg) // subscribe해 온 msg를 _laserCloud에 저장
  {
    _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp; 

    laserCloud()->clear();
    pcl::fromROSMsg(*laserCloudFullResMsg, *laserCloud());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloud(), *laserCloud(), indices);

    //std::cout << "lasercloud-odometry.rgb: " << laserCloud()->r << std::cout;
    _newLaserCloudFullRes = true;
  }

  void LaserOdometry::imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg)
  {
    _timeImuTrans = imuTransMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZ> imuTrans; // pcl point형식으로 imutrans용 변수를 새로 선언.

    pcl::fromROSMsg(*imuTransMsg, imuTrans); // subscribe해 온 msg를 새로 만든 변수에 저장.
    updateIMU(imuTrans);                     // 하나의 변수로 보내진 정보들을 4종류의 변수에 나누어 저장한다.
    _newImuTrans = true;
  }


  void LaserOdometry::spin()
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
  }


  bool LaserOdometry::hasNewData()
  {
    return _newCornerPointsSharp && _newCornerPointsLessSharp && _newSurfPointsFlat &&
      _newSurfPointsLessFlat && _newLaserCloudFullRes && _newImuTrans &&
      fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
      fabs((_timeImuTrans - _timeSurfPointsLessFlat).toSec()) < 0.005;
  }// 새로운 데이터가 들어오면 true 아니면 false



  void LaserOdometry::process()
  {
    if (!hasNewData()) // 새로운 데이터가 없으면 process하지 않음. (hasNewData: 새로운 데이터가 있으면 true 없으면 false)
      return; // waiting for new data to arrive...

    reset(); // reset flags, etc.   // 데이터가 들어오면 이전 process에서 이전 것들을 reset후,
    BasicLaserOdometry::process();  // BasicLaserOdometry의 process실행.

    publishResult();                // LaserOdometry 노드의 결과물들을 publish.
  }


  void LaserOdometry::publishResult() // 결과 데이터를 message(_laserOdometryMsg)로 포장해서 LaserMapping 노드나 TrasformMaintainance 노드로 publish.
  {
    // publish odometry transformations. (_transformSum)
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum().rot_z.rad(),
                                                                               -transformSum().rot_x.rad(),
                                                                               -transformSum().rot_y.rad());  // 최적화된 pose 변환행렬의 축척 = _transformSum의 rotation 정보로부터 Quaternion을 생성.

    _laserOdometryMsg.header.stamp            = _timeSurfPointsLessFlat;  // time stamp.
    _laserOdometryMsg.pose.pose.orientation.x = -geoQuat.y;               // rotation 정보.
    _laserOdometryMsg.pose.pose.orientation.y = -geoQuat.z;
    _laserOdometryMsg.pose.pose.orientation.z = geoQuat.x;
    _laserOdometryMsg.pose.pose.orientation.w = geoQuat.w;
    _laserOdometryMsg.pose.pose.position.x    = transformSum().pos.x();   // translation 정보.
    _laserOdometryMsg.pose.pose.position.y    = transformSum().pos.y();
    _laserOdometryMsg.pose.pose.position.z    = transformSum().pos.z();
    _pubLaserOdometry.publish(_laserOdometryMsg);                         // publish.

    // 위와 같은 laserOdometry 변환행렬을 tf로도 broadcast한다.
    _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat;                                                               // time stamp.
    _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));                      // rotation 정보.
    _laserOdometryTrans.setOrigin(tf::Vector3(transformSum().pos.x(), transformSum().pos.y(), transformSum().pos.z())); // translation 정보.
    _tfBroadcaster.sendTransform(_laserOdometryTrans);                                                                  // _laserOdometryTrans를 tf broadcast.
    //로봇(또는 자율주행)의 경우 굉장히 많은 3차원 좌표계로 구성되어있고, 이는 시간에 걸쳐 계속 변화한다. 
    //자율주행에선 예로 World 프레임, Base 프레임, 각종 Sensor 프레임 등이 있다.
    // tf는 각 좌표계를 지속적으로 추적하기 때문에 tf를 통해 각 프레임 간 관계를 이용할 수 있다


    // publish cloud results according to the input output ratio.   // 입출력 비율에 따라 포인트 클라우드 메세지들(3개)을 publish.
    if (_ioRatio < 2 || frameCount() % _ioRatio == 1)
    {
      ros::Time sweepTime = _timeSurfPointsLessFlat;
      
      publishCloudMsg(_pubLaserCloudCornerLast, *lastCornerCloud(), sweepTime, "/camera");  // 모서리 클라우드.
      publishCloudMsg(_pubLaserCloudSurfLast, *lastSurfaceCloud(), sweepTime, "/camera");   // 평면 클라우드.

      transformToEnd(laserCloud());  // transform full resolution cloud to sweep end before sending it. // full해상도 포인트 클라우드는 보내기 전, sweep의 끝 시점으로 투영해 주어야 함.
      publishCloudMsg(_pubLaserCloudFullRes, *laserCloud(), sweepTime, "/camera"); // full해상도 포인트 클라우드.
    }
  }

} // end namespace loam
