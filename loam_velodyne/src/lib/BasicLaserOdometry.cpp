#include "loam_velodyne/BasicLaserOdometry.h"

#include "math_utils.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

///////////////////////
// 모든 .intensity를 .curvature로 바꿈
///////////////////////

namespace loam
{

using std::sin;
using std::cos;
using std::asin;
using std::atan2;
using std::sqrt;
using std::fabs;
using std::pow;


BasicLaserOdometry::BasicLaserOdometry(float scanPeriod, size_t maxIterations) :
   _scanPeriod(scanPeriod), //time per period
   _systemInited(false),       ///< initialization flag
   _frameCount(0),            ///< number of processed frames
   _maxIterations(maxIterations), ///< maximum number of iterations
   _deltaTAbort(0.1),            ///< optimization abort threshold for deltaT
   _deltaRAbort(0.1),            ///< optimization abort threshold for deltaR
   _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),   //edge
   _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZRGBNormal>()), //less edge
   _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),       // planar
   _surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),   // less planar
   _laserCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),    // 이전 edge
   _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),   // 이전 planar
   _laserCloudOri(new pcl::PointCloud<pcl::PointXYZRGBNormal>()),
   _coeffSel(new pcl::PointCloud<pcl::PointXYZRGBNormal>())
{}



void BasicLaserOdometry::transformToStart(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGBNormal& po)
{
   float s = (1.f / _scanPeriod) * (pi.curvature - int(pi.curvature));  // 1/주기 = 주파수 , 주파수 * 빛의 세기(원래 값- 정수 값을 빼줘서 소수 부분만 ) 

   po.x = pi.x - s * _transform.pos.x(); //위치 계산 움직인 거리 추정?
   po.y = pi.y - s * _transform.pos.y();
   po.z = pi.z - s * _transform.pos.z();
   po.curvature = pi.curvature;

   Angle rx = -s * _transform.rot_x.rad();  // 각도 
   Angle ry = -s * _transform.rot_y.rad();
   Angle rz = -s * _transform.rot_z.rad();
   rotateZXY(po, rz, rx, ry);
}



size_t BasicLaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud)
{
   size_t cloudSize = cloud->points.size();

   for (size_t i = 0; i < cloudSize; i++)
   {
      pcl::PointXYZRGBNormal& point = cloud->points[i];

      float s = (1.f / _scanPeriod) * (point.curvature - int(point.curvature));

      point.x -= s * _transform.pos.x();
      point.y -= s * _transform.pos.y();
      point.z -= s * _transform.pos.z();
      point.curvature = (int(point.curvature));

      Angle rx = -s * _transform.rot_x.rad();
      Angle ry = -s * _transform.rot_y.rad();
      Angle rz = -s * _transform.rot_z.rad();
      rotateZXY(point, rz, rx, ry);
      rotateYXZ(point, _transform.rot_y, _transform.rot_x, _transform.rot_z);

      point.x += _transform.pos.x() - _imuShiftFromStart.x();
      point.y += _transform.pos.y() - _imuShiftFromStart.y();
      point.z += _transform.pos.z() - _imuShiftFromStart.z();

      rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart);
      rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);
   }

   return cloudSize;
}



void BasicLaserOdometry::pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                                           const Angle& blx, const Angle& bly, const Angle& blz,
                                           const Angle& alx, const Angle& aly, const Angle& alz,
                                           Angle &acx, Angle &acy, Angle &acz)
{
   float sbcx = bcx.sin();
   float cbcx = bcx.cos();
   float sbcy = bcy.sin();
   float cbcy = bcy.cos();
   float sbcz = bcz.sin();
   float cbcz = bcz.cos();

   float sblx = blx.sin();
   float cblx = blx.cos();
   float sbly = bly.sin();
   float cbly = bly.cos();
   float sblz = blz.sin();
   float cblz = blz.cos();

   float salx = alx.sin();
   float calx = alx.cos();
   float saly = aly.sin();
   float caly = aly.cos();
   float salz = alz.sin();
   float calz = alz.cos();

   float srx = -sbcx * (salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly)
      - cbcx * cbcz*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                     - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - cbcx * sbcz*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                     - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz);
   acx = -asin(srx);

   float srycrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                                  - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                        - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      + cbcx * sbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   float crycrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                                  - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                        - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      + cbcx * cbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

   float srzcrx = sbcx * (cblx*cbly*(calz*saly - caly * salx*salz) - cblx * sbly*(caly*calz + salx * saly*salz) + calx * salz*sblx)
      - cbcx * cbcz*((caly*calz + salx * saly*salz)*(cbly*sblz - cblz * sblx*sbly)
                     + (calz*saly - caly * salx*salz)*(sbly*sblz + cbly * cblz*sblx)
                     - calx * cblx*cblz*salz)
      + cbcx * sbcz*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
                     + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz)
                     + calx * cblx*salz*sblz);
   float crzcrx = sbcx * (cblx*sbly*(caly*salz - calz * salx*saly) - cblx * cbly*(saly*salz + caly * calz*salx) + calx * calz*sblx)
      + cbcx * cbcz*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
                     + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly)
                     + calx * calz*cblx*cblz)
      - cbcx * sbcz*((saly*salz + caly * calz*salx)*(cblz*sbly - cbly * sblx*sblz)
                     + (caly*salz - calz * salx*saly)*(cbly*cblz + sblx * sbly*sblz)
                     - calx * calz*cblx*sblz);
   acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
}



void BasicLaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,
                                            Angle lx, Angle ly, Angle lz,
                                            Angle &ox, Angle &oy, Angle &oz)
{
   float srx = lx.cos()*cx.cos()*ly.sin()*cz.sin()
      - cx.cos()*cz.cos()*lx.sin()
      - lx.cos()*ly.cos()*cx.sin();
   ox = -asin(srx);

   float srycrx = lx.sin()*(cy.cos()*cz.sin() - cz.cos()*cx.sin()*cy.sin())
      + lx.cos()*ly.sin()*(cy.cos()*cz.cos() + cx.sin()*cy.sin()*cz.sin())
      + lx.cos()*ly.cos()*cx.cos()*cy.sin();
   float crycrx = lx.cos()*ly.cos()*cx.cos()*cy.cos()
      - lx.cos()*ly.sin()*(cz.cos()*cy.sin() - cy.cos()*cx.sin()*cz.sin())
      - lx.sin()*(cy.sin()*cz.sin() + cy.cos()*cz.cos()*cx.sin());
   oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

   float srzcrx = cx.sin()*(lz.cos()*ly.sin() - ly.cos()*lx.sin()*lz.sin())
      + cx.cos()*cz.sin()*(ly.cos()*lz.cos() + lx.sin()*ly.sin()*lz.sin())
      + lx.cos()*cx.cos()*cz.cos()*lz.sin();
   float crzcrx = lx.cos()*lz.cos()*cx.cos()*cz.cos()
      - cx.cos()*cz.sin()*(ly.cos()*lz.sin() - lz.cos()*lx.sin()*ly.sin())
      - cx.sin()*(ly.sin()*lz.sin() + ly.cos()*lz.cos()*lx.sin());
   oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}


void BasicLaserOdometry::updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans) // imu값 업데이트
{
   assert(4 == imuTrans.size());
   _imuPitchStart = imuTrans.points[0].x;
   _imuYawStart = imuTrans.points[0].y;
   _imuRollStart = imuTrans.points[0].z;

   _imuPitchEnd = imuTrans.points[1].x;
   _imuYawEnd = imuTrans.points[1].y;
   _imuRollEnd = imuTrans.points[1].z;

   _imuShiftFromStart = imuTrans.points[2];
   _imuVeloFromStart = imuTrans.points[3];
}


// /////////////////// 중복 제거 알고리즘 ///////////////////
// bool BasicLaserOdometry::isOverlap(const pcl::PointXYZRGB& point){
  
//   // 중복 제거 알고리즘 용.
  
//   int preci = 1000;  // mm 범위에 대해 반올림.
//   float tmpArr[3];
//   std::string tmpString;
  
//   // 1. cm 범위에 대해 반올림. (100을 곱한 뒤, round()후 다시 100을 나눈다.)
//   tmpArr[0] = round(point.x * preci) / preci;
//   tmpArr[1] = round(point.y * preci) / preci;
//   tmpArr[2] = round(point.z * preci) / preci;

//   // 2. 각 좌표를 string으로 바꾼 후, 하나의 string으로 합침.
//   tmpString = std::to_string(tmpArr[0]) + ',' + std::to_string(tmpArr[1]) + ',' + std::to_string(tmpArr[2]);

//   // 3. 중복된 좌표인지 체크
//   iter = overlapCheck.find(tmpString);

//   // 4. 만일 존재하는 경우 continue
//   if(iter != overlapCheck.end())
//     return true;

//   // 5. 존재하지 않는경우 set에 추가.
//   overlapCheck.insert(tmpString);

//   return false;
// }

// ////////////// 임의의 픽셀 포인트 추가 알고리즘 //////////////
// void BasicLaserOdometry::makePixelPoint(const pcl::PointXYZRGB& point, int scanID){
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
//       thresh = (2*M_PI/dist);
//       maxRadius = 0.001;
      
//       for(float r = dist; r <= maxRadius; r += dist){
        
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
//                   _pixelCloud.push_back(pixPoint); 
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


//LaserOdometry.cpp의 process함수에서 basiclaserodomety의 process함수를 부름
void BasicLaserOdometry::process()
{
   if (!_systemInited)     // 만일 초기화가 되지않았다면 아래와 같이 초기화한 후 return한다.
   {
      _cornerPointsLessSharp.swap(_lastCornerCloud);
      _surfPointsLessFlat.swap(_lastSurfaceCloud);

      _lastCornerKDTree.setInputCloud(_lastCornerCloud);    // 빠른 search를 위해 KD tree에 이전 scan의 특징점들을 넣는다.
      _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);

      _transformSum.rot_x += _imuPitchStart;    // transformSum에 이전 imu state를 세팅한다.
      _transformSum.rot_z += _imuRollStart;

      _systemInited = true;
      return;
   }

   // ////////////// 임의의 픽셀 포인트 추가 알고리즘 //////////////
   // pcl::PointXYZRGB point;
   // float lowerBound = -15;
   // float upperBound = 15;
   // uint16_t nScanRings = 16;
   // float _factor = (nScanRings - 1) / (upperBound - lowerBound);

   // for (auto const& pt : _laserCloud->points) {
   //    if(pt.normal_x != 1 && pt.normal_y != 1)  // 컬러가 없는 포인트들은 넘어감.
   //       continue;

   //    point.x = pt.x;
   //    point.y = pt.y;
   //    point.z = pt.z;
   //    //point.rgb = pt.rgb;

   //    // calculate vertical point angle and scan ID.                                       // 포인트의 세로 각과 어떤 layer에서 스캔되었는지 계산한다.
   //    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z)); // 세로각.
   //    int scanID = int(((angle * 180 / M_PI) - lowerBound) * _factor + 0.5);                                   // 어떤 layer에서 스캔되었는지.

   //    if (scanID >= nScanRings || scanID < 0 ){                    // 만일 layer가 사용하는 라이다의 layer갯수 이상이거나 0보다 작다면 건너뛴다.
   //       continue;
   //    }

   //    makePixelPoint(point, scanID);

   // }
   // ////////////////////////////////////////////////////////

   pcl::PointXYZRGBNormal coeff;
   bool isDegenerate = false;
   Eigen::Matrix<float, 6, 6> matP;

   _frameCount++;
   _transform.pos -= _imuVeloFromStart * _scanPeriod;


   size_t lastCornerCloudSize = _lastCornerCloud->points.size();     // 두 특징점 클라우드의 사이즈 변수 선언.
   size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

   if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)    // 특징점들이 일정 갯수 이상 검출되었을 때에 아래의 코드 실행.
   {
      std::vector<int> pointSearchInd(1);
      std::vector<float> pointSearchSqDis(1);
      std::vector<int> indices;

      pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);
      size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();
      size_t surfPointsFlatNum = _surfPointsFlat->points.size();

      _pointSearchCornerInd1.resize(cornerPointsSharpNum);
      _pointSearchCornerInd2.resize(cornerPointsSharpNum);
      _pointSearchSurfInd1.resize(surfPointsFlatNum);
      _pointSearchSurfInd2.resize(surfPointsFlatNum);
      _pointSearchSurfInd3.resize(surfPointsFlatNum);

      for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++)
      {
         pcl::PointXYZRGBNormal pointSel, pointProj, tripod1, tripod2, tripod3;
         _laserCloudOri->clear();
         _coeffSel->clear();

         for (int i = 0; i < cornerPointsSharpNum; i++)
         {
            transformToStart(_cornerPointsSharp->points[i], pointSel);  // k+1번째 sweep에서 검출된 corner 특징점을, k번째 특징점들과 비교를 위해 sweep의 시작지점으로 투영(reproject).
                                                                        // 투영엔 이전 sweep에서 생성한, '최적화된 변환행렬'인 _transform를 이용한다.

            if (iterCount % 5 == 0)
            {
               pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices);
               _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);    // 각 특징점과 가장 가까운 이전 scan의 특징점을 search.

               int closestPointInd = -1, minPointInd2 = -1;
               if (pointSearchSqDis[0] < 25)
               {
                  closestPointInd = pointSearchInd[0];
                  int closestPointScan = int((_lastCornerCloud->points[closestPointInd].curvature));
                  float pointSqDis, minPointSqDis2 = 25;
                  for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
                  {
                     if (int((_lastCornerCloud->points[j].curvature)) > closestPointScan + 2.5) 
                     {
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

                     if (int((_lastCornerCloud->points[j].curvature)) > closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2)
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                  }
                  for (int j = closestPointInd - 1; j >= 0; j--)
                  {
                     if (int((_lastCornerCloud->points[j].curvature)) < closestPointScan - 2.5)
                     {
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

                     if (int((_lastCornerCloud->points[j].curvature)) < closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2)
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                  }
               }

               _pointSearchCornerInd1[i] = closestPointInd;
               _pointSearchCornerInd2[i] = minPointInd2;
            }

            if (_pointSearchCornerInd2[i] >= 0)
            {
               tripod1 = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
               tripod2 = _lastCornerCloud->points[_pointSearchCornerInd2[i]];

               float x0 = pointSel.x;
               float y0 = pointSel.y;
               float z0 = pointSel.z;
               float x1 = tripod1.x;
               float y1 = tripod1.y;
               float z1 = tripod1.z;
               float x2 = tripod2.x;
               float y2 = tripod2.y;
               float z2 = tripod2.z;

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

               float ld2 = a012 / l12; // Eq. (2)

               // TODO: Why writing to a variable that's never read?
               pointProj = pointSel;
               pointProj.x -= la * ld2;
               pointProj.y -= lb * ld2;
               pointProj.z -= lc * ld2;
               
            
               float s = 1;
               if (iterCount >= 5)
               {
                  s = 1 - 1.8f * fabs(ld2);
               }

               coeff.x = s * la;
               coeff.y = s * lb;
               coeff.z = s * lc;
               coeff.curvature = s * ld2;

               if (s > 0.1 && ld2 != 0)
               {
                  _laserCloudOri->push_back(_cornerPointsSharp->points[i]);   // 각 특징점과 가장 가까운 neighbor point를 골라 _laserCloudOri에 저장.
                  _coeffSel->push_back(coeff);
               }
            }
         }     // corner 특징점 비교 완료.

         for (int i = 0; i < surfPointsFlatNum; i++)
         {
            transformToStart(_surfPointsFlat->points[i], pointSel);  // k+1번째 sweep에서 검출된 flat 특징점을, k번째 특징점들과 비교를 위해 sweep의 시작지점으로 투영(reproject).
                                                                     // 투영엔 이전 sweep에서 생성한, '최적화된 변환행렬'인 _transform를 이용한다.

            if (iterCount % 5 == 0)
            {
               _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);    // 각 특징점과 가장 가까운 이전 scan의 특징점을 search.
               int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
               if (pointSearchSqDis[0] < 25)
               {
                  closestPointInd = pointSearchInd[0];
                  int closestPointScan = int((_lastSurfaceCloud->points[closestPointInd].curvature));

                  float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
                  for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
                  {
                     if (int((_lastSurfaceCloud->points[j].curvature)) > closestPointScan + 2.5)
                     {
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

                     if (int((_lastSurfaceCloud->points[j].curvature)) <= closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2)
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                     else
                     {
                        if (pointSqDis < minPointSqDis3)
                        {
                           minPointSqDis3 = pointSqDis;
                           minPointInd3 = j;
                        }
                     }
                  }
                  for (int j = closestPointInd - 1; j >= 0; j--)
                  {
                     if (int((_lastSurfaceCloud->points[j].curvature)) < closestPointScan - 2.5)
                     {
                        break;
                     }

                     pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

                     if (int((_lastSurfaceCloud->points[j].curvature)) >= closestPointScan)
                     {
                        if (pointSqDis < minPointSqDis2)
                        {
                           minPointSqDis2 = pointSqDis;
                           minPointInd2 = j;
                        }
                     }
                     else
                     {
                        if (pointSqDis < minPointSqDis3)
                        {
                           minPointSqDis3 = pointSqDis;
                           minPointInd3 = j;
                        }
                     }
                  }
               }

               _pointSearchSurfInd1[i] = closestPointInd;
               _pointSearchSurfInd2[i] = minPointInd2;
               _pointSearchSurfInd3[i] = minPointInd3;
            }

            if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0)
            {
               tripod1 = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
               tripod2 = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
               tripod3 = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

               float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                  - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
               float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                  - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
               float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                  - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
               float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

               float ps = sqrt(pa * pa + pb * pb + pc * pc);
               pa /= ps;
               pb /= ps;
               pc /= ps;
               pd /= ps;

               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd; //Eq. (3)??

               // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
               pointProj = pointSel;
               pointProj.x -= pa * pd2;
               pointProj.y -= pb * pd2;
               pointProj.z -= pc * pd2;

               float s = 1;
               if (iterCount >= 5)
               {
                  s = 1 - 1.8f * fabs(pd2) / sqrt(calcPointDistance(pointSel));
               }

               coeff.x = s * pa;
               coeff.y = s * pb;
               coeff.z = s * pc;
               coeff.curvature = (s * pd2);

               if (s > 0.1 && pd2 != 0)
               {
                  _laserCloudOri->push_back(_surfPointsFlat->points[i]);   // 각 특징점과 가장 가까운 neighbor point를 골라 _laserCloudOri에 저장.
                  _coeffSel->push_back(coeff);
               }
            }
         }     // flat 특징점 비교 완료.

         int pointSelNum = _laserCloudOri->points.size();   // pointSelNum = number of selected points = 골라진 포인트들의 총 갯수.
         if (pointSelNum < 10)
         {
            continue;
         }


         ////////////////////////////  변환행렬 T구하기  ////////////////////////////
         // 위에서 KD tree를 이용해 고른 포인트들을 이용해, d 를 최소화 하는 변환행렬 T를 구한다.
         Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);
         Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum);
         Eigen::Matrix<float, 6, 6> matAtA;
         Eigen::VectorXf matB(pointSelNum);
         Eigen::Matrix<float, 6, 1> matAtB;
         Eigen::Matrix<float, 6, 1> matX;

         for (int i = 0; i < pointSelNum; i++)  // 각 point들을 이용해 matrix A와 matrix B 생성. 
         {
            const pcl::PointXYZRGBNormal& pointOri = _laserCloudOri->points[i];
            coeff = _coeffSel->points[i];

            float s = 1;

            float srx = sin(s * _transform.rot_x.rad());
            float crx = cos(s * _transform.rot_x.rad());
            float sry = sin(s * _transform.rot_y.rad());
            float cry = cos(s * _transform.rot_y.rad());
            float srz = sin(s * _transform.rot_z.rad());
            float crz = cos(s * _transform.rot_z.rad());
            float tx = s * _transform.pos.x();
            float ty = s * _transform.pos.y();
            float tz = s * _transform.pos.z();

            float arx = (-s * crx*sry*srz*pointOri.x + s * crx*crz*sry*pointOri.y + s * srx*sry*pointOri.z
                         + s * tx*crx*sry*srz - s * ty*crx*crz*sry - s * tz*srx*sry) * coeff.x
               + (s*srx*srz*pointOri.x - s * crz*srx*pointOri.y + s * crx*pointOri.z
                  + s * ty*crz*srx - s * tz*crx - s * tx*srx*srz) * coeff.y
               + (s*crx*cry*srz*pointOri.x - s * crx*cry*crz*pointOri.y - s * cry*srx*pointOri.z
                  + s * tz*cry*srx + s * ty*crx*cry*crz - s * tx*crx*cry*srz) * coeff.z;

            float ary = ((-s * crz*sry - s * cry*srx*srz)*pointOri.x
                         + (s*cry*crz*srx - s * sry*srz)*pointOri.y - s * crx*cry*pointOri.z
                         + tx * (s*crz*sry + s * cry*srx*srz) + ty * (s*sry*srz - s * cry*crz*srx)
                         + s * tz*crx*cry) * coeff.x
               + ((s*cry*crz - s * srx*sry*srz)*pointOri.x
                  + (s*cry*srz + s * crz*srx*sry)*pointOri.y - s * crx*sry*pointOri.z
                  + s * tz*crx*sry - ty * (s*cry*srz + s * crz*srx*sry)
                  - tx * (s*cry*crz - s * srx*sry*srz)) * coeff.z;

            float arz = ((-s * cry*srz - s * crz*srx*sry)*pointOri.x + (s*cry*crz - s * srx*sry*srz)*pointOri.y
                         + tx * (s*cry*srz + s * crz*srx*sry) - ty * (s*cry*crz - s * srx*sry*srz)) * coeff.x
               + (-s * crx*crz*pointOri.x - s * crx*srz*pointOri.y
                  + s * ty*crx*srz + s * tx*crx*crz) * coeff.y
               + ((s*cry*crz*srx - s * sry*srz)*pointOri.x + (s*crz*sry + s * cry*srx*srz)*pointOri.y
                  + tx * (s*sry*srz - s * cry*crz*srx) - ty * (s*crz*sry + s * cry*srx*srz)) * coeff.z;

            float atx = -s * (cry*crz - srx * sry*srz) * coeff.x + s * crx*srz * coeff.y
               - s * (crz*sry + cry * srx*srz) * coeff.z;

            float aty = -s * (cry*srz + crz * srx*sry) * coeff.x - s * crx*crz * coeff.y
               - s * (sry*srz - cry * crz*srx) * coeff.z;

            float atz = s * crx*sry * coeff.x - s * srx * coeff.y - s * crx*cry * coeff.z;

            float d2 = (coeff.curvature);

            matA(i, 0) = arx;
            matA(i, 1) = ary;
            matA(i, 2) = arz;
            matA(i, 3) = atx;
            matA(i, 4) = aty;
            matA(i, 5) = atz;
            matB(i, 0) = -0.05 * d2;
         }  

         // matrix X 생성.
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
            float eignThre[6] = { 10, 10, 10, 10, 10, 10 };
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
         
         // matrix X를 이용해 변환행렬 T = _transform 을 update. 
         _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
         _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
         _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
         _transform.pos.x() += matX(3, 0);
         _transform.pos.y() += matX(4, 0);
         _transform.pos.z() += matX(5, 0);

         if (!pcl_isfinite(_transform.rot_x.rad())) _transform.rot_x = Angle();
         if (!pcl_isfinite(_transform.rot_y.rad())) _transform.rot_y = Angle();
         if (!pcl_isfinite(_transform.rot_z.rad())) _transform.rot_z = Angle();

         if (!pcl_isfinite(_transform.pos.x())) _transform.pos.x() = 0.0;
         if (!pcl_isfinite(_transform.pos.y())) _transform.pos.y() = 0.0;
         if (!pcl_isfinite(_transform.pos.z())) _transform.pos.z() = 0.0;

         float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                             pow(rad2deg(matX(1, 0)), 2) +
                             pow(rad2deg(matX(2, 0)), 2));
         float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                             pow(matX(4, 0) * 100, 2) +
                             pow(matX(5, 0) * 100, 2));

         if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
            break;
         ////////////////////////////  변환행렬 T구하기  ////////////////////////////
      }
   } // end of if

   //////////////////  변환행렬 T의 축적인 _transformSum 를 업데이트  //////////////////
   Angle rx, ry, rz;
   accumulateRotation(_transformSum.rot_x,
                      _transformSum.rot_y,
                      _transformSum.rot_z,
                      -_transform.rot_x,
                      -_transform.rot_y.rad() * 1.05,
                      -_transform.rot_z,
                      rx, ry, rz);     // 변수 rx, ry, rz에 imu state의 rotation을 제외한 현재까지의 rotation 정보 축적.

   Vector3 v(_transform.pos.x() - _imuShiftFromStart.x(),
             _transform.pos.y() - _imuShiftFromStart.y(),
             _transform.pos.z() * 1.05 - _imuShiftFromStart.z()); // 변환행렬 T와 imu state의 translation을 반영한 변수 v 생성.

   rotateZXY(v, rz, rx, ry);  // 좌표계를 올바르게 변환.

   Vector3 trans = _transformSum.pos - v; // 변수 trans에 현재까지의 translation 정보 축적.

   pluginIMURotation(rx, ry, rz,
                     _imuPitchStart, _imuYawStart, _imuRollStart,
                     _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                     rx, ry, rz);   // 변수 rx, ry, rz에 추가적으로 imu state의 rotation 정보 축적.


   // _transformSum를 최종적으로 update.
   _transformSum.rot_x = rx;
   _transformSum.rot_y = ry;
   _transformSum.rot_z = rz;
   _transformSum.pos = trans;
   //////////////////  변환행렬 T의 축적인 _transformSum 를 업데이트  //////////////////


   // k+2 번째 sweep의 포인트 클라우드와 비교하기 위해 현재(k+1) 생성이 끝난 포인트 클라우드를 k+2시점으로 투영(project).
   transformToEnd(_cornerPointsLessSharp);   
   transformToEnd(_surfPointsLessFlat);


   // 또한 다음번(k+2) process를 위해 initialization을 진행. //
   _cornerPointsLessSharp.swap(_lastCornerCloud);
   _surfPointsLessFlat.swap(_lastSurfaceCloud);

   lastCornerCloudSize = _lastCornerCloud->points.size();
   lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

   if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)
   {
      _lastCornerKDTree.setInputCloud(_lastCornerCloud);
      _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
   }
   //********************************************************//
}



} // end namespace loam
