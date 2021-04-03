#pragma once 
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


#include "Twist.h"
#include "CircularBuffer.h"
#include "time_utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cv.hpp>

#include <cmath>
#include <queue>
#include <set>
#include <string>
#include <cv_bridge/cv_bridge.h>

#define PCNUM 100000 // number of _laserCloudFullResColorStack point clouds

using namespace cv;

namespace loam
{

/** IMU 상태 데이터에 대한 구조체 / IMU state data. */
typedef struct IMUState2
{
   /** 이 상태가 되기 까지의 시간 / The time of the measurement leading to this state (in seconds). */
   Time stamp;

   /** 현재 roll 각도 / The current roll angle. */
   Angle roll;

   /** 현재 pitch 각도 / The current pitch angle. */
   Angle pitch;

   /** \brief Interpolate between two IMU states. 두 IMU 상태 사이의 보간(interpolation)
    *
    * @param start 첫번째 IMU 상태 / the first IMU state
    * @param end 첫번째 IMU 상태 / the second IMU state
    * @param ratio 보간의 비율 / the interpolation ratio
    * @param result 보간 결과를 저장하기 위한 객체 / the target IMU state for storing the interpolation result
    */
   static void interpolate(const IMUState2& start,
                           const IMUState2& end,
                           const float& ratio,
                           IMUState2& result)
   {
      float invRatio = 1 - ratio;

      result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
      result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
   };
} IMUState2;



class BasicLaserMapping
{
public:
   explicit BasicLaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief 새롭게 저장된 데이터를 처리하는 메소드 / Try to process buffered data. */
   bool process(Time const& laserOdometryTime);

   void updateIMU(IMUState2 const& newState);
   void updateOdometry(double pitch, double yaw, double roll, double x, double y, double z);
   void updateOdometry(Twist const& twist);

   auto& laserCloud() { return *_laserCloudFullRes; }
   auto& laserCloudCornerLast() { return *_laserCloudCornerLast; }
   auto& laserCloudSurfLast() { return *_laserCloudSurfLast; }

   void setScanPeriod(float val) { _scanPeriod = val; }
   void setMaxIterations(size_t val) { _maxIterations = val; }
   void setDeltaTAbort(float val) { _deltaTAbort = val; }
   void setDeltaRAbort(float val) { _deltaRAbort = val; }

   auto& downSizeFilterCorner() { return _downSizeFilterCorner; }
   auto& downSizeFilterSurf() { return _downSizeFilterSurf; }
   auto& downSizeFilterMap() { return _downSizeFilterMap; }

   auto frameCount()    const { return _frameCount; }
   auto scanPeriod()    const { return _scanPeriod; }
   auto maxIterations() const { return _maxIterations; }
   auto deltaTAbort()   const { return _deltaTAbort; }
   auto deltaRAbort()   const { return _deltaRAbort; }

   auto const& transformAftMapped()   const { return _transformAftMapped; }
   auto const& transformBefMapped()   const { return _transformBefMapped; }
   auto const& laserCloudSurroundDS() const { return *_laserCloudSurroundDS; }

   bool hasFreshMap() const { return _downsizedMapCreated; }


   ////////////////////////////////////////////////////////////////////
   void updateZedPose(double pitch, double yaw, double roll, double x, double y, double z);
   
   auto const& laserCloudSurround() const { return _laserCloudSurround; }
   // auto const& laserCloudSurround() const { return *_laserCloudSurround; }
   
   auto const& laserCloudMap() const { return *_laserCloudMap; }
   
   auto const& zedWorldTrans()   const { return _zedWorldTrans; }

   void changetoZedRT(Twist const& twist);

   // auto const& laserCloudSurroundColor() const { return _laserCloudSurroundColor; }
   ////////////////////////////////////////////////////////////////////


   ////////////////////////////////////////////////////////////////////
   bool isOverlap(const pcl::PointXYZRGB& point);

   //HD 내부파라미터
   cv::Mat K;
   cv::Mat E = (cv::Mat_<float>(3,4) <<  -1,  0, 0, 0.165, //0.06 ,0.15, 0.165
                                          0, -1, 0, 0.066, //-0.056, -0.026, 0.066
                                          0,  0, 1, 0.0444); //0.0444
   cv::Mat KE;

   float* depths;
   cv::Mat _mat_left;
   cv::Mat _mat_depth;
   ////////////////////////////////////////////////////////////////////
   

private:
   /** 매핑을 위해 변환행렬(_transformTobeMapped)을 최적화 하는 메소드 / Run an optimization. */
   void optimizeTransformTobeMapped();

   void transformAssociateToMap();

   // optimizeTransformTobeMapped() 마지막에 다음 sweep을 위해 변환행렬 변수 2개를 update하는 메소드. 이때 만일 imu state가 존재(사용가능)하면, update하기 전 imu state를 이용해 더욱 최적화 한다.
   void transformUpdate();    

   void pointAssociateToMap(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGBNormal& po);
   void pointAssociateToMap(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGB& po);
   void pointAssociateToMap(const pcl::PointXYZRGB& pi, pcl::PointXYZRGB& po);
   void pointAssociateTobeMapped(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGBNormal& po);
   void pointAssociateTobeMapped(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGB& po);

   void transformFullResToMap();

   bool createDownsizedMap();

   // private:
   size_t toIndex(int i, int j, int k) const
   { return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k; }

private:
   Time _laserOdometryTime;

   float _scanPeriod;          ///< 스캔 당 시간 / time per scan
   const int _stackFrameNum;
   const int _mapFrameNum;
   long _frameCount;
   long _mapFrameCount;

   size_t _maxIterations;  ///< iteration 최대 수 / maximum number of iterations
   float _deltaTAbort;     ///< delta T에 대한 최적의 threshold 값 / optimization abort threshold for deltaT
   float _deltaRAbort;     ///< delta R에 대한 최적의 threshold 값 / optimization abort threshold for deltaR

   int _laserCloudCenWidth;
   int _laserCloudCenHeight;
   int _laserCloudCenDepth;
   const size_t _laserCloudWidth;
   const size_t _laserCloudHeight;
   const size_t _laserCloudDepth;
   const size_t _laserCloudNum;

   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudCornerLast;   ///< last corner points cloud
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudSurfLast;     ///< last surface points cloud
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

   /////////////////////////////////////////////////
   pcl::PointCloud<pcl::PointXYZRGB> _laserCloudToViewer;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr _laserCloudSurround;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr _laserCloudMap;
   std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _laserCloudMapArray;
   // pcl::PointCloud<pcl::PointXYZRGB> _laserCloudFullResColorStack;
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr _laserCloudSurroundColor;
   
   int SInd = 0;
   int toZeroCnt = 0;
   Twist _zedWorldTrans;         // subscribe로 계속해서 업데이트 되는 변수.

   std::set<std::string> overlapCheck;
   std::set<std::string>::iterator iter;

   bool _newPointCloud = false;     // 클라우드의 업데이트를 확인하기 위한 flag.
   bool _newLaserCloudMap = false;  // 축적된 map을 publish 하기 위한 flag.
   //////////////////////////////////////////////////////

   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudCornerStack;
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudSurfStack;
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudCornerStackDS;  ///< down sampled
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudSurfStackDS;    ///< down sampled

   
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudSurroundDS;     ///< down sampled
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudCornerFromMap;
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudSurfFromMap;

   pcl::PointCloud<pcl::PointXYZRGBNormal> _laserCloudOri;
   pcl::PointCloud<pcl::PointXYZRGBNormal> _coeffSel;

   std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> _laserCloudCornerArray;
   std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> _laserCloudSurfArray;
   std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> _laserCloudCornerDSArray;  ///< down sampled
   std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> _laserCloudSurfDSArray;    ///< down sampled

   std::vector<size_t> _laserCloudValidInd;
   std::vector<size_t> _laserCloudSurroundInd;

   // 변환 행렬들을 위한 변수들.
   Twist _transformSum, _transformIncre, _transformTobeMapped, _transformBefMapped, _transformAftMapped;
   // _transformSum        = subscribe해 온 현재 sweep의 odometry정보. 즉 (T^L)_k+1
   // _transformBefMapped  = 이전 sweep의 _transformSum.
   // _transformIncre      = _transformBefMapped와 _transformSum의 translation 차이값. 즉 k+1과 k+2 사이의 translation.
   // _transformTobeMapped = 새로운 pc를 기존의 map에 이어 붙이기 위한 변환행렬. 즉 (T^L)_k+1 과 (T^W)_k로 생성되는 (T^W)_k+1.
   // _transformAftMapped  = 이전 sweep의 _transformTobeMapped. 즉 (T^W)_k

   CircularBuffer<IMUState2> _imuHistory;    ///< IMU상태에 대한 과거 기록들 / history of IMU states

   pcl::VoxelGrid<pcl::PointXYZRGBNormal> _downSizeFilterCorner;   ///< 코너 클라우드를 다운사이징하기 위한 복셀 필터 / voxel filter for down sizing corner clouds
   pcl::VoxelGrid<pcl::PointXYZRGBNormal> _downSizeFilterSurf;     ///< 표면 클라우드를 다운사이징하기 위한 복셀 필터 / voxel filter for down sizing surface clouds
   pcl::VoxelGrid<pcl::PointXYZRGBNormal> _downSizeFilterMap;      ///< 누적 맵 크기를 줄이기 위한 복셀 필터 / voxel filter for down sizing accumulated map

   bool _downsizedMapCreated = false;
};

} // end namespace loam





