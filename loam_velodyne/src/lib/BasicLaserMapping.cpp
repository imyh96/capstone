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

#include "loam_velodyne/BasicLaserMapping.h"
#include "loam_velodyne/nanoflann_pcl.h"
#include "math_utils.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>


#define lidarRT 0
#define camRT 1

int cnt = 0;


namespace loam
{

using std::sqrt;
using std::fabs;
using std::asin;
using std::atan2;
using std::pow;


BasicLaserMapping::BasicLaserMapping(const float& scanPeriod, const size_t& maxIterations) :
   _scanPeriod(scanPeriod),
   _stackFrameNum(1),
   _mapFrameNum(1),  // 5 -> 1
   _frameCount(0),
   _mapFrameCount(0),
   _maxIterations(maxIterations),
   _deltaTAbort(0.05),
   _deltaRAbort(0.05),
   _laserCloudCenWidth(10),
   _laserCloudCenHeight(5),
   _laserCloudCenDepth(10),
   _laserCloudWidth(21),
   _laserCloudHeight(11),
   _laserCloudDepth(21),
   _laserCloudNum(_laserCloudWidth * _laserCloudHeight * _laserCloudDepth),
   _laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudCornerStackDS(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudSurfStackDS(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudSurround(new pcl::PointCloud<pcl::PointXYZRGB>()),  //PointXYZRGBNormal
   _laserCloudSurroundDS(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZRGBNormal>())
{
   // frame 수 세는 변수(초기화) / initialize frame counter
   _frameCount = _stackFrameNum - 1;
   _mapFrameCount = _mapFrameNum - 1;

   // 클라우드 벡터들 setup / setup cloud vectors
   _laserCloudCornerArray.resize(_laserCloudNum);
   _laserCloudSurfArray.resize(_laserCloudNum);
   _laserCloudCornerDSArray.resize(_laserCloudNum);
   _laserCloudSurfDSArray.resize(_laserCloudNum);

   //////////////////////////
   _laserCloudFullResArray.resize(PCNUM);
   for(int i = 0; i < PCNUM; i++)
   {
      _laserCloudFullResArray[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>());
   }
   //////////////////////////

   for (size_t i = 0; i < _laserCloudNum; i++)
   {
      _laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      _laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      _laserCloudCornerDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      _laserCloudSurfDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
   }

   // 축소화 시키는 필터 setup / setup down size filters
   _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
   _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
}

void BasicLaserMapping::transformAssociateToMap()
{
   // _transformSum        = subscribe해 온 현재 sweep의 odometry정보. 즉 (T^L)_k+1
   // _transformBefMapped  = 이전 sweep의 _transformSum.
   // _transformIncre      = _transformBefMapped와 _transformSum의 translation 차이값. 즉 k+1과 k+2 사이의 translation.
   // _transformTobeMapped = (T^L)_k+1 과 (T^W)_k로 생성할 (T^W)_k+1.
   // _transformAftMapped  = 이전 sweep의 _transformTobeMapped. 즉 (T^W)_k

   _transformIncre.pos = _transformBefMapped.pos - _transformSum.pos;
   rotateYXZ(_transformIncre.pos, -(_transformSum.rot_y), -(_transformSum.rot_x), -(_transformSum.rot_z));

   float sbcx = _transformSum.rot_x.sin();
   float cbcx = _transformSum.rot_x.cos();
   float sbcy = _transformSum.rot_y.sin();
   float cbcy = _transformSum.rot_y.cos();
   float sbcz = _transformSum.rot_z.sin();
   float cbcz = _transformSum.rot_z.cos();

   float sblx = _transformBefMapped.rot_x.sin();
   float cblx = _transformBefMapped.rot_x.cos();
   float sbly = _transformBefMapped.rot_y.sin();
   float cbly = _transformBefMapped.rot_y.cos();
   float sblz = _transformBefMapped.rot_z.sin();
   float cblz = _transformBefMapped.rot_z.cos();

   float salx = _transformAftMapped.rot_x.sin();
   float calx = _transformAftMapped.rot_x.cos();
   float saly = _transformAftMapped.rot_y.sin();
   float caly = _transformAftMapped.rot_y.cos();
   float salz = _transformAftMapped.rot_z.sin();
   float calz = _transformAftMapped.rot_z.cos();

   float srx = -sbcx * (salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz)
      - cbcx * sbcy*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                     - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      - cbcx * cbcy*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                     - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx);
   _transformTobeMapped.rot_x = -asin(srx);

   float srycrx = sbcx * (cblx*cblz*(caly*salz - calz * salx*saly)
                          - cblx * sblz*(caly*calz + salx * saly*salz) + calx * saly*sblx)
      - cbcx * cbcy*((caly*calz + salx * saly*salz)*(cblz*sbly - cbly * sblx*sblz)
                     + (caly*salz - calz * salx*saly)*(sbly*sblz + cbly * cblz*sblx) - calx * cblx*cbly*saly)
      + cbcx * sbcy*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
                     + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly) + calx * cblx*saly*sbly);
   float crycrx = sbcx * (cblx*sblz*(calz*saly - caly * salx*salz)
                          - cblx * cblz*(saly*salz + caly * calz*salx) + calx * caly*sblx)
      + cbcx * cbcy*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
                     + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz) + calx * caly*cblx*cbly)
      - cbcx * sbcy*((saly*salz + caly * calz*salx)*(cbly*sblz - cblz * sblx*sbly)
                     + (calz*saly - caly * salx*salz)*(cbly*cblz + sblx * sbly*sblz) - calx * caly*cblx*sbly);
   _transformTobeMapped.rot_y = atan2(srycrx / _transformTobeMapped.rot_x.cos(),
                                      crycrx / _transformTobeMapped.rot_x.cos());

   float srzcrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                                                  - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                                        - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      + cbcx * sbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   float crzcrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
                                                  - calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
                                        - calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
      + cbcx * cbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
   _transformTobeMapped.rot_z = atan2(srzcrx / _transformTobeMapped.rot_x.cos(),
                                      crzcrx / _transformTobeMapped.rot_x.cos());

   Vector3 v = _transformIncre.pos;
   rotateZXY(v, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);
   _transformTobeMapped.pos = _transformAftMapped.pos - v;
}

void BasicLaserMapping::transformUpdate() // 다음 sweep을 위해 변환행렬 변수들을 update한다.
{
   if (0 < _imuHistory.size())   // 만일 imu state가 존재(사용가능)하면, update하기 전 imu state를 이용해 더욱 최적화 한다.
   {
      size_t imuIdx = 0;

      while (imuIdx < _imuHistory.size() - 1 && toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0)
      {
         imuIdx++;
      }

      IMUState2 imuCur;

      if (imuIdx == 0 || toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0)
      {
         // scan time newer then newest or older than oldest IMU message
         imuCur = _imuHistory[imuIdx];
      }
      else
      {
         float ratio = (toSec(_imuHistory[imuIdx].stamp - _laserOdometryTime) - _scanPeriod)
            / toSec(_imuHistory[imuIdx].stamp - _imuHistory[imuIdx - 1].stamp);

         IMUState2::interpolate(_imuHistory[imuIdx], _imuHistory[imuIdx - 1], ratio, imuCur);
      }

      _transformTobeMapped.rot_x = 0.998 * _transformTobeMapped.rot_x.rad() + 0.002 * imuCur.pitch.rad();
      _transformTobeMapped.rot_z = 0.998 * _transformTobeMapped.rot_z.rad() + 0.002 * imuCur.roll.rad();
   }

   // 다음 sweep을 위해 변환행렬 변수들을 update한다.
   _transformBefMapped = _transformSum;
   _transformAftMapped = _transformTobeMapped;
}

void BasicLaserMapping::pointAssociateToMap(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGB& po)
{
   po.x = pi.x;
   po.y = pi.y;
   po.z = pi.z;

   ////////////////  // 변환할 때 rgb값과 normal값도 함께 넘겨주어야 함.
   // po.normal_x = pi.normal_x;
   // po.normal_y = pi.normal_y;
   // po.normal_z = pi.normal_z;
   
   po.rgb = pi.rgb;
   ////////////////

   // po.curvature = pi.curvature;

   rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

   po.x += _transformTobeMapped.pos.x();
   po.y += _transformTobeMapped.pos.y();
   po.z += _transformTobeMapped.pos.z();
}
void BasicLaserMapping::pointAssociateToMap(const pcl::PointXYZRGB& pi, pcl::PointXYZRGB& po)
{
   po.x = pi.x;
   po.y = pi.y;
   po.z = pi.z;

   ////////////////  // 변환할 때 rgb값과 normal값도 함께 넘겨주어야 함.
   // po.normal_x = pi.normal_x;
   // po.normal_y = pi.normal_y;
   // po.normal_z = pi.normal_z;
   
   po.rgb = pi.rgb;
   ////////////////

   // po.curvature = pi.curvature;

   rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

   po.x += _transformTobeMapped.pos.x();
   po.y += _transformTobeMapped.pos.y();
   po.z += _transformTobeMapped.pos.z();
}
void BasicLaserMapping::pointAssociateToMap(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGBNormal& po)
{
   po.x = pi.x;
   po.y = pi.y;
   po.z = pi.z;

   //////////////  // 변환할 때 rgb값과 normal값도 함께 넘겨주어야 함.
   po.normal_x = pi.normal_x;
   po.normal_y = pi.normal_y;
   po.normal_z = pi.normal_z;
   
   po.rgb = pi.rgb;
   ////////////////

   po.curvature = pi.curvature;

   rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

   po.x += _transformTobeMapped.pos.x();
   po.y += _transformTobeMapped.pos.y();
   po.z += _transformTobeMapped.pos.z();
}
void BasicLaserMapping::pointAssociateTobeMapped(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGBNormal& po)
{
   po.x = pi.x - _transformTobeMapped.pos.x();
   po.y = pi.y - _transformTobeMapped.pos.y();
   po.z = pi.z - _transformTobeMapped.pos.z();

   ////////////////  // 변환할 때 rgb값과 normal값도 함께 넘겨주어야 함.
   po.normal_x = pi.normal_x;
   po.normal_y = pi.normal_y;
   po.normal_z = pi.normal_z;

   po.rgb = pi.rgb;
   ////////////////
   
   po.curvature = pi.curvature;

   rotateYXZ(po, -_transformTobeMapped.rot_y, -_transformTobeMapped.rot_x, -_transformTobeMapped.rot_z);
}
void BasicLaserMapping::pointAssociateTobeMapped(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGB& po)
{
   po.x = pi.x - _transformTobeMapped.pos.x();
   po.y = pi.y - _transformTobeMapped.pos.y();
   po.z = pi.z - _transformTobeMapped.pos.z();

   // ////////////////  // 변환할 때 rgb값과 normal값도 함께 넘겨주어야 함.
   // po.normal_x = pi.normal_x;
   // po.normal_y = pi.normal_y;
   // po.normal_z = pi.normal_z;

   po.rgb = pi.rgb;
   ////////////////
   
   // po.curvature = pi.curvature;

   rotateYXZ(po, -_transformTobeMapped.rot_y, -_transformTobeMapped.rot_x, -_transformTobeMapped.rot_z);
}

void BasicLaserMapping::transformFullResToMap()
{
   // transform full resolution input cloud to map.      // 현재 스캔중인 full resolution cloud를 보여주는 함수
   for (auto& pt : *_laserCloudFullRes){
      pointAssociateToMap(pt, pt);
   }
}

bool BasicLaserMapping::createDownsizedMap()
{
   // create new map cloud according to the input output ratio.      // 입력, 출력비에 따라 새로운 맵 클라우드를 생성 
   _mapFrameCount++;
   if (_mapFrameCount < _mapFrameNum)
      return false;

   _mapFrameCount = 0;

   // accumulate map cloud.      // map cloud를 축적한다.
   _laserCloudSurround->clear();
   *_laserCloudSurround += _laserCloudFullResColor;

   newPointCloud = true;

   // // down size map cloud.    // map cloud를 다운 사이즈한다.
   // _laserCloudSurroundDS->clear();
   // _downSizeFilterCorner.setInputCloud(_laserCloudSurround);
   // _downSizeFilterCorner.filter(*_laserCloudSurroundDS);

   // _sweepCnt++;
   // std::cout << "_sweepCnt: " << _sweepCnt << std::endl;
   return true;
}



/////////////////// 중복 제거 알고리즘 ///////////////////
bool BasicLaserMapping::isOverlap(const pcl::PointXYZRGB& point){
  
   // 중복 제거 알고리즘 용.

   int preci = 500;  // mm 범위에 대해 반올림.
   float tmpArr[3];
   std::string tmpString;

   // 1. mm 범위에 대해 반올림. (preci를 곱한 뒤, round()후 다시 preci를 나눈다.)
   tmpArr[0] = round(point.x * preci) / preci;
   tmpArr[1] = round(point.y * preci) / preci;
   tmpArr[2] = round(point.z * preci) / preci;

   // 2. 각 좌표를 string으로 바꾼 후, 하나의 string으로 합침.
   tmpString = std::to_string(tmpArr[0]) + ',' + std::to_string(tmpArr[1]) + ',' + std::to_string(tmpArr[2]);

   // 3. 중복된 좌표인지 체크
   iter = overlapCheck.find(tmpString);

   // 4. 만일 존재하는 경우 continue
   if(iter != overlapCheck.end()){
      //cnt++;
      //cout << "cnt: " << cnt << '\n';
      return true;
   }
      

   // 5. 존재하지 않는경우 set에 추가.
   overlapCheck.insert(tmpString);

   return false;
}


bool BasicLaserMapping::process(Time const& laserOdometryTime)
{
   // skip some frames?!?     // 프레임 count가 _stackFrameNum보다 작은 경우엔 스킵한다.
   _frameCount++;
   if (_frameCount < _stackFrameNum)
   {
      return false;
   }
   _frameCount = 0;

   _laserOdometryTime = laserOdometryTime;   // time stamp.

   pcl::PointXYZRGBNormal pointSel;

   ///////////////////////////
   pcl::PointXYZRGB pointRGB;    // PointXYZRGBNormal -> PointXYZRGB 타입으로 바꿔주기 위한 포인트 변수.
   ///////////////////////////


   // // relate incoming data to map.     // 새로 subscribe한 transform인 _transformSum = (T^L)_k+1 와 이전 sweep에서 만든 (T^W)_k로,
   //                                     // 새로운 pc를 기존의 map에 이어 붙이기 위한 변환행렬인 (T^W)_k+1 = _transformTobeMapped 를 생성하는 함수.
#if lidarRT   
   transformAssociateToMap();
#endif

#if camRT
   /////////////////////////////////////////////////////
   changetoZedRT(_zedWorldTrans);   // A를 생성하기위한 현재 sweep의 trans 저장.
   /////////////////////////////////////////////////////
#endif
   

   // 새로 subscribe 해 온 두 특징점 클라우드(_laserCloudCornerLast,_laserCloudSurfLast)의 각 포인트들을 복사해서, 
   // 복사한 점들을 _transformTobeMapped를 이용해 기존 map 좌표계 상에 투영(project)한 것들의 클라우드인 _laserCloudCornerStack와 _laserCloudSurfStack를 생성.
   for (auto const& pt : _laserCloudCornerLast->points)
   {
      pointAssociateToMap(pt, pointSel);
      _laserCloudCornerStack->push_back(pointSel);
   }

   for (auto const& pt : _laserCloudSurfLast->points)
   {
      pointAssociateToMap(pt, pointSel);
      _laserCloudSurfStack->push_back(pointSel);
   }


   ///////////////////////  map 클라우드의 특징점들 주위 영역 내에 있는 point들로 이루어진 surround cloud 생성  ///////////////////////

   // Y축 상의 점 하나 생성.
   pcl::PointXYZRGBNormal pointOnYAxis;
   pointOnYAxis.x = 0.0;
   pointOnYAxis.y = 10.0;
   pointOnYAxis.z = 0.0;
   pointAssociateToMap(pointOnYAxis, pointOnYAxis);

   auto const CUBE_SIZE = 50.0;
   auto const CUBE_HALF = CUBE_SIZE / 2;

   int centerCubeI = int((_transformTobeMapped.pos.x() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
   int centerCubeJ = int((_transformTobeMapped.pos.y() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
   int centerCubeK = int((_transformTobeMapped.pos.z() + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

   if (_transformTobeMapped.pos.x() + CUBE_HALF < 0) centerCubeI--;
   if (_transformTobeMapped.pos.y() + CUBE_HALF < 0) centerCubeJ--;
   if (_transformTobeMapped.pos.z() + CUBE_HALF < 0) centerCubeK--;

   while (centerCubeI < 3)
   {
      for (int j = 0; j < _laserCloudHeight; j++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int i = _laserCloudWidth - 1; i >= 1; i--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i - 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(0, j, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeI++;
      _laserCloudCenWidth++;
   }

   while (centerCubeI >= _laserCloudWidth - 3)
   {
      for (int j = 0; j < _laserCloudHeight; j++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int i = 0; i < _laserCloudWidth - 1; i++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i + 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(_laserCloudWidth - 1, j, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeI--;
      _laserCloudCenWidth--;
   }

   while (centerCubeJ < 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = _laserCloudHeight - 1; j >= 1; j--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j - 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, 0, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeJ++;
      _laserCloudCenHeight++;
   }

   while (centerCubeJ >= _laserCloudHeight - 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = 0; j < _laserCloudHeight - 1; j++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j + 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, _laserCloudHeight - 1, k);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeJ--;
      _laserCloudCenHeight--;
   }

   while (centerCubeK < 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = _laserCloudDepth - 1; k >= 1; k--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k - 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, j, 0);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeK++;
      _laserCloudCenDepth++;
   }

   while (centerCubeK >= _laserCloudDepth - 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = 0; k < _laserCloudDepth - 1; k++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k + 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
            const size_t indexC = toIndex(i, j, _laserCloudDepth - 1);
            _laserCloudCornerArray[indexC]->clear();
            _laserCloudSurfArray[indexC]->clear();
         }
      }
      centerCubeK--;
      _laserCloudCenDepth--;
   }
   
   _laserCloudValidInd.clear();
   _laserCloudSurroundInd.clear();
   for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
   {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
         for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
         {
            if (i >= 0 && i < _laserCloudWidth &&
                j >= 0 && j < _laserCloudHeight &&
                k >= 0 && k < _laserCloudDepth)
            {

               float centerX = 50.0f * (i - _laserCloudCenWidth);
               float centerY = 50.0f * (j - _laserCloudCenHeight);
               float centerZ = 50.0f * (k - _laserCloudCenDepth);

               pcl::PointXYZRGBNormal transform_pos = (pcl::PointXYZRGBNormal) _transformTobeMapped.pos;

               bool isInLaserFOV = false;
               for (int ii = -1; ii <= 1; ii += 2)
               {
                  for (int jj = -1; jj <= 1; jj += 2)
                  {
                     for (int kk = -1; kk <= 1; kk += 2)
                     {
                        pcl::PointXYZRGBNormal corner;
                        corner.x = centerX + 25.0f * ii;
                        corner.y = centerY + 25.0f * jj;
                        corner.z = centerZ + 25.0f * kk;

                        float squaredSide1 = calcSquaredDiff(transform_pos, corner);
                        float squaredSide2 = calcSquaredDiff(pointOnYAxis, corner);

                        float check1 = 100.0f + squaredSide1 - squaredSide2
                           - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                        float check2 = 100.0f + squaredSide1 - squaredSide2
                           + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                        if (check1 < 0 && check2 > 0)
                        {
                           isInLaserFOV = true;
                        }
                     }
                  }
               }

               size_t cubeIdx = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
               if (isInLaserFOV)
               {
                  _laserCloudValidInd.push_back(cubeIdx);
               }
               _laserCloudSurroundInd.push_back(cubeIdx);      // surround cloud index 생성.
            }
         }
      }
   }
   //////////////////////////////////////////////////////////////////////


   ///////////////////////////////////////////////////////////////////////////////////////////////////////////
   // 새롭게 subscribe해온 특징점들중 valid한 것들을 이용해 최적화된 변환행렬인 _transformTobeMapped를 구한다.//
   // prepare valid map corner and surface cloud for pose optimization.     // 포즈 최적화를 위해 '유효(valid)한 map cloud의 특징점(corner & surface)들의 클라우드'를 준비
   _laserCloudCornerFromMap->clear();
   _laserCloudSurfFromMap->clear();
   for (auto const& ind : _laserCloudValidInd)
   {
      *_laserCloudCornerFromMap += *_laserCloudCornerArray[ind];
      *_laserCloudSurfFromMap += *_laserCloudSurfArray[ind];
   }

   // prepare feature stack clouds for pose optimization.      // 포즈 최적화를 위해 '새로 입력된 cloud의 특징점 클라우드(feature stack cloud)'를 준비 .
                                                               // ( 위에서 구한 _transformTobeMapped를 이용해 월드 좌표계로 이동시켜 둔다 = pointAssociateTobeMapped() ).
   for (auto& pt : *_laserCloudCornerStack)
      pointAssociateTobeMapped(pt, pt);

   for (auto& pt : *_laserCloudSurfStack)
      pointAssociateTobeMapped(pt, pt);

   // down sample feature stack clouds.      // feature stack cloud들을 down sizing한다.
   _laserCloudCornerStackDS->clear();
   _downSizeFilterCorner.setInputCloud(_laserCloudCornerStack);
   _downSizeFilterCorner.filter(*_laserCloudCornerStackDS);
   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();     // pose 최적화 함수는 이렇게 down sizing된 클라우드를 사용한다.

   _laserCloudSurfStackDS->clear();
   _downSizeFilterSurf.setInputCloud(_laserCloudSurfStack);
   _downSizeFilterSurf.filter(*_laserCloudSurfStackDS);
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();     // pose 최적화 함수는 이렇게 down sizing된 클라우드를 사용한다.

   _laserCloudCornerStack->clear();
   _laserCloudSurfStack->clear();

#if lidarRT 
   // run pose optimization.        // 포즈 최적화를 시행한다.
   optimizeTransformTobeMapped();   // Odometry 알고리즘과 같이, 3차원 KD tree를 이용해 d 를 최소화 하는 변환행렬(_transformTobeMapped)을 구한다.
   /////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif


   ////////////////////////////////////////
  int xp, yp, Idx, B, G, R;
  
  uchar* p;
  int channels = _mat_left.channels();

  float depthZ, depthL, xl, yl, zl;
  
//   int zedepth, lidepth;
//   float zedepthf, lidepthf;
  std::uint32_t rgb;

  double fx_d = K.at<float>(0,0);
  double fy_d = K.at<float>(1,1);
  double cx_d = K.at<float>(0,2);
  double cy_d = K.at<float>(1,2);

  int pixRange = 1; // 1 = 3x3, 2 = 5x5, 3 = 7x7 

  pcl::PointXYZRGB pixpoint;

  int ind = 0;
  float range = 0.2;
  ////////////////////////////////////////
   
   
   ///////////////////////////////////////////////////////
   // full res pc의 포인트들 중, 컬러가 입혀진 포인트들만 world좌표계로 변환, 포인트 타입 변환 후, _laserCloudFullResColorStack에 축적한다. 
   _laserCloudFullResColor.clear();
   for (auto& pt : *_laserCloudFullRes){
      
      // 해당 점에 색이 입혀진 경우.
      if(pt.normal_z != -1 && pt.normal_y != -1 && pt.normal_x != -1){

         xl = pt.x - 0.165; // 라이다 점의 depth값을 구할 때, 하드웨어의 위치관계를 고려해 주어야 한다.
         yl = pt.y + 0.066;
         zl = pt.z - 0.0444;

         depthL = sqrt(xl*xl + yl*yl + zl*zl);
         depthZ = pt.normal_z;
         xp = pt.normal_x;
         yp = pt.normal_y;

         if(((depthL-range) < depthZ) && (depthZ < (depthL+range)))
         {
            for(int x = xp-pixRange; x < xp+pixRange; x++){
               for(int y = yp-pixRange; y < yp+pixRange; y++){

                  ind = x + 1280 * y;

                  if(((depthL-range) < depths[ind]) && (depths[ind] < (depthL+range)))
                  {
                     pixpoint.x = -((x - cx_d) * depthL / fx_d);
                     pixpoint.y = -((y - cy_d) * depthL / fy_d);
                     pixpoint.z = depthL;

                     if(!isOverlap(pixpoint))
                     {
                        p = _mat_left.ptr<uchar>(y);
                        B = p[x*channels + 0];   // left 이미지에서 컬러값 추출.
                        G = p[x*channels + 1];
                        R = p[x*channels + 2]; 

                        rgb = (static_cast<std::uint32_t>(R) << 16 | static_cast<std::uint32_t>(G) << 8 | static_cast<std::uint32_t>(B));
                        pixpoint.rgb = *reinterpret_cast<float*>(&rgb);

                        pointAssociateToMap(pixpoint, pointRGB);
                        
                        _laserCloudFullResColor.push_back(pointRGB);
                        
                        if(SInd > PCNUM-1)
                           SInd = 0;
                        _laserCloudFullResArray[SInd]->push_back(pointRGB);

                     }   
                     
                  }
                  
               }
            }
         }
      }  
   }

   SInd++;
   ///////////////////////////////////////////////////////
   

   // store down sized corner stack points in corresponding cube clouds.      // 대응하는 cube 클라우드에 축소(down size)된 corner stack points를 다시 map으로 위치 이동후 저장.
   for (int i = 0; i < laserCloudCornerStackNum; i++)
   {
      pointAssociateToMap(_laserCloudCornerStackDS->points[i], pointSel);

      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
         _laserCloudCornerArray[cubeInd]->push_back(pointSel);
      }
   }
   // store down sized surface stack points in corresponding cube clouds.     // 대응하는 cube 클라우드에 축소(down size)된 surface stack points를 다시 map으로 위치 이동후 저장.
   for (int i = 0; i < laserCloudSurfStackNum; i++)
   {
      pointAssociateToMap(_laserCloudSurfStackDS->points[i], pointSel);

      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
         _laserCloudSurfArray[cubeInd]->push_back(pointSel);
      }
   }

   


   // down size all valid (within field of view) feature cube clouds.      // 모든 유효(FOV 내에 존재함)한 feature cube cloud들을 down sizing한다. 
   for (auto const& ind : _laserCloudValidInd)
   {
      _laserCloudCornerDSArray[ind]->clear();
      _downSizeFilterCorner.setInputCloud(_laserCloudCornerArray[ind]);
      _downSizeFilterCorner.filter(*_laserCloudCornerDSArray[ind]);

      _laserCloudSurfDSArray[ind]->clear();
      _downSizeFilterSurf.setInputCloud(_laserCloudSurfArray[ind]);
      _downSizeFilterSurf.filter(*_laserCloudSurfDSArray[ind]);

      // swap cube clouds for next processing
      _laserCloudCornerArray[ind].swap(_laserCloudCornerDSArray[ind]);
      _laserCloudSurfArray[ind].swap(_laserCloudSurfDSArray[ind]);
   }


   // odometry로 부터 subscribe하여 저장해 둔 full resolution 포인트 클라우드를, 최척화 한 변환행렬을 이용해 현재 라이다의 pose에 맞춰 보여준다.
   transformFullResToMap();

   // 포인트들을 고르게 분포시키기 위해 map 클라우드를 복셀 사이즈가 (5cm)^3인 voxel grid 필터로 다운 사이징한, map을 생성한다.
   _downsizedMapCreated = createDownsizedMap();

   // // _transformTobeMapped의 내부 값들 확인용
   // ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
   //          "LIDAR",
   //          _transformTobeMapped.pos.x(), _transformTobeMapped.pos.y(), _transformTobeMapped.pos.z(),
   //          _transformTobeMapped.rot_x.deg(), _transformTobeMapped.rot_y.deg(), _transformTobeMapped.rot_z.deg());


   return true;
}  
// process() finish




void BasicLaserMapping::updateIMU(IMUState2 const& newState)
{
   _imuHistory.push(newState);
}

void BasicLaserMapping::updateOdometry(double pitch, double yaw, double roll, double x, double y, double z)
{
   _transformSum.rot_x = pitch;
   _transformSum.rot_y = yaw;
   _transformSum.rot_z = roll;

   _transformSum.pos.x() = float(x);
   _transformSum.pos.y() = float(y);
   _transformSum.pos.z() = float(z);
}

/////////////////////////////////////////////////////////
void BasicLaserMapping::updateZedPose(double pitch, double yaw, double roll, double x, double y, double z)
{
   _zedWorldTrans.rot_x = pitch;
   _zedWorldTrans.rot_y = yaw;
   _zedWorldTrans.rot_z = roll;

   _zedWorldTrans.pos.x() = float(y);
   _zedWorldTrans.pos.y() = float(z);
   _zedWorldTrans.pos.z() = float(x);
}

void BasicLaserMapping::changetoZedRT(Twist const& twist){
   _transformTobeMapped = twist;
}


void BasicLaserMapping::updateOdometry(Twist const& twist)
{
   _transformSum = twist;
}

nanoflann::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtreeCornerFromMap;
nanoflann::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtreeSurfFromMap;



// pose 최적화용 함수.
void BasicLaserMapping::optimizeTransformTobeMapped()
{
   if (_laserCloudCornerFromMap->size() <= 10 || _laserCloudSurfFromMap->size() <= 100)
      return;

   pcl::PointXYZRGBNormal pointSel, pointOri, /*pointProj, */coeff;

   std::vector<int> pointSearchInd(5, 0);
   std::vector<float> pointSearchSqDis(5, 0);

   kdtreeCornerFromMap.setInputCloud(_laserCloudCornerFromMap);
   kdtreeSurfFromMap.setInputCloud(_laserCloudSurfFromMap);

   Eigen::Matrix<float, 5, 3> matA0;
   Eigen::Matrix<float, 5, 1> matB0;
   Eigen::Vector3f matX0;
   Eigen::Matrix3f matA1;
   Eigen::Matrix<float, 1, 3> matD1;
   Eigen::Matrix3f matV1;

   matA0.setZero();
   matB0.setConstant(-1);
   matX0.setZero();

   matA1.setZero();
   matD1.setZero();
   matV1.setZero();

   bool isDegenerate = false;
   Eigen::Matrix<float, 6, 6> matP;

   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();

   for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++)
   {
      _laserCloudOri.clear();
      _coeffSel.clear();

      for (int i = 0; i < laserCloudCornerStackNum; i++)
      {
         pointOri = _laserCloudCornerStackDS->points[i];
         pointAssociateToMap(pointOri, pointSel);
         kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

         if (pointSearchSqDis[4] < 1.0)
         {
            Vector3 vc(0, 0, 0);

            for (int j = 0; j < 5; j++)
               vc += Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]);
            vc /= 5.0;

            Eigen::Matrix3f mat_a;
            mat_a.setZero();

            for (int j = 0; j < 5; j++)
            {
               Vector3 a = Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]) - vc;

               mat_a(0, 0) += a.x() * a.x();
               mat_a(1, 0) += a.x() * a.y();
               mat_a(2, 0) += a.x() * a.z();
               mat_a(1, 1) += a.y() * a.y();
               mat_a(2, 1) += a.y() * a.z();
               mat_a(2, 2) += a.z() * a.z();
            }
            matA1 = mat_a / 5.0;
            // This solver only looks at the lower-triangular part of matA1.     // 이 솔버는 matA1의 아래쪽 삼각형 부분만 본다. 
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
            matD1 = esolver.eigenvalues().real();
            matV1 = esolver.eigenvectors().real();

            if (matD1(0, 2) > 3 * matD1(0, 1))
            {

               float x0 = pointSel.x;
               float y0 = pointSel.y;
               float z0 = pointSel.z;
               float x1 = vc.x() + 0.1 * matV1(0, 2);
               float y1 = vc.y() + 0.1 * matV1(1, 2);
               float z1 = vc.z() + 0.1 * matV1(2, 2);
               float x2 = vc.x() - 0.1 * matV1(0, 2);
               float y2 = vc.y() - 0.1 * matV1(1, 2);
               float z2 = vc.z() - 0.1 * matV1(2, 2);

               float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                 * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

               float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

               float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

               float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float ld2 = a012 / l12;

//                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
//                pointProj = pointSel;
//                pointProj.x -= la * ld2;
//                pointProj.y -= lb * ld2;
//                pointProj.z -= lc * ld2;

               float s = 1 - 0.9f * fabs(ld2);

               coeff.x = s * la;
               coeff.y = s * lb;
               coeff.z = s * lc;
               coeff.curvature = (s * ld2);// * FI_COEF;

               if (s > 0.1)
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }
         }
      }

      for (int i = 0; i < laserCloudSurfStackNum; i++)
      {
         pointOri = _laserCloudSurfStackDS->points[i];
         pointAssociateToMap(pointOri, pointSel);
         kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

         if (pointSearchSqDis[4] < 1.0)
         {
            for (int j = 0; j < 5; j++)
            {
               matA0(j, 0) = _laserCloudSurfFromMap->points[pointSearchInd[j]].x;
               matA0(j, 1) = _laserCloudSurfFromMap->points[pointSearchInd[j]].y;
               matA0(j, 2) = _laserCloudSurfFromMap->points[pointSearchInd[j]].z;
            }
            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
               if (fabs(pa * _laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        pb * _laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        pc * _laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
               {
                  planeValid = false;
                  break;
               }
            }

            if (planeValid)
            {
               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

               //                // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
               //                pointProj = pointSel;
               //                pointProj.x -= pa * pd2;
               //                pointProj.y -= pb * pd2;
               //                pointProj.z -= pc * pd2;

               float s = 1 - 0.9f * fabs(pd2) / sqrt(calcPointDistance(pointSel));

               coeff.x = s * pa;
               coeff.y = s * pb;
               coeff.z = s * pc;
               coeff.curvature = s * pd2; //* FI_COEF;

               if (s > 0.1)
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }
         }
      }

      float srx = _transformTobeMapped.rot_x.sin();
      float crx = _transformTobeMapped.rot_x.cos();
      float sry = _transformTobeMapped.rot_y.sin();
      float cry = _transformTobeMapped.rot_y.cos();
      float srz = _transformTobeMapped.rot_z.sin();
      float crz = _transformTobeMapped.rot_z.cos();

      size_t laserCloudSelNum = _laserCloudOri.size();
      if (laserCloudSelNum < 50)
         continue;

      Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
      Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
      Eigen::Matrix<float, 6, 6> matAtA;
      Eigen::VectorXf matB(laserCloudSelNum);
      Eigen::VectorXf matAtB;
      Eigen::VectorXf matX;

      for (int i = 0; i < laserCloudSelNum; i++)
      {
         pointOri = _laserCloudOri.points[i];
         coeff = _coeffSel.points[i];

         float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
            + (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
            + (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

         float ary = ((cry*srx*srz - crz * sry)*pointOri.x
                      + (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
            + ((-cry * crz - srx * sry*srz)*pointOri.x
               + (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

         float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
            + (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
            + ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

         matA(i, 0) = arx;
         matA(i, 1) = ary;
         matA(i, 2) = arz;
         matA(i, 3) = coeff.x;
         matA(i, 4) = coeff.y;
         matA(i, 5) = coeff.z;
         matB(i, 0) = -(coeff.curvature );/// FI_COEF
      }

      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;
      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      if (iterCount == 0)
      {
         Eigen::Matrix<float, 1, 6> matE;
         Eigen::Matrix<float, 6, 6> matV;
         Eigen::Matrix<float, 6, 6> matV2;

         Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
         matE = esolver.eigenvalues().real();
         matV = esolver.eigenvectors().real();

         matV2 = matV;

         isDegenerate = false;
         float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
         for (int i = 0; i < 6; i++)
         {
            if (matE(0, i) < eignThre[i])
            {
               for (int j = 0; j < 6; j++)
               {
                  matV2(i, j) = 0;
               }
               isDegenerate = true;
            }
            else
            {
               break;
            }
         }
         matP = matV.inverse() * matV2;
      }

      if (isDegenerate)
      {
         Eigen::Matrix<float, 6, 1> matX2(matX);
         matX = matP * matX2;
      }

      _transformTobeMapped.rot_x += matX(0, 0);
      _transformTobeMapped.rot_y += matX(1, 0);
      _transformTobeMapped.rot_z += matX(2, 0);
      _transformTobeMapped.pos.x() += matX(3, 0);
      _transformTobeMapped.pos.y() += matX(4, 0);
      _transformTobeMapped.pos.z() += matX(5, 0);

      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
         break;
   }

   transformUpdate();
}


} // end namespace loam
