#include <pcl/filters/voxel_grid.h>

#include "loam_velodyne/BasicScanRegistration.h"
#include "math_utils.h"

namespace loam
{

RegistrationParams::RegistrationParams(const float& scanPeriod_,
                                       const int& imuHistorySize_,
                                       const int& nFeatureRegions_,
                                       const int& curvatureRegion_,
                                       const int& maxCornerSharp_,
                                       const int& maxSurfaceFlat_,
                                       const float& lessFlatFilterSize_,
                                       const float& surfaceCurvatureThreshold_)
    : scanPeriod(scanPeriod_),
      imuHistorySize(imuHistorySize_),
      nFeatureRegions(nFeatureRegions_),
      curvatureRegion(curvatureRegion_),
      maxCornerSharp(maxCornerSharp_),
      maxCornerLessSharp(10 * maxCornerSharp_),
      maxSurfaceFlat(maxSurfaceFlat_),
      lessFlatFilterSize(lessFlatFilterSize_),
      surfaceCurvatureThreshold(surfaceCurvatureThreshold_)
{};

void BasicScanRegistration::processScanlines(const Time& scanTime, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> const& laserCloudScans)
{
  // reset internal buffers and set IMU start state based on current scan time.   // 이전 스캔에서 생성했던 포인트 클라우드들을 리셋하고 현재 스캔이 수행되는 시간(scanTime)을 기반으로 IMU start state를 새롭게 세팅한다.
  reset(scanTime);  

  // construct sorted full resolution cloud.                                      // 정렬된 full resolution cloud인 _laserCloud를 생성한다.
  size_t cloudSize = 0;

  for (int i = 0; i < laserCloudScans.size(); i++) {
    _laserCloud += laserCloudScans[i];

    IndexRange range(cloudSize, 0);
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

  extractFeatures();    // 현재 scan의 특징점들을 추출한다. 즉 _cornerPointsSharp, _cornerPointsLessSharp, _surfacePointsFlat 그리고 _surfacePointsLessFlat를 생성한다.
  updateIMUTransform(); // IMU 변환행렬을 담고있는 변수인 _imuTrans를 생성한다.
}

bool BasicScanRegistration::configure(const RegistrationParams& config)
{
  _config = config;
  _imuHistory.ensureCapacity(_config.imuHistorySize);
  return true;
}

void BasicScanRegistration::reset(const Time& scanTime)
{
  _scanTime = scanTime;

  // re-initialize IMU start index and state                   // IMU의 인덱스와 state를 다시 초기화 한다.
  _imuIdx = 0;
  if (hasIMUData()) {
    interpolateIMUStateFor(0, _imuStart);
  }

  // clear internal cloud buffers at the beginning of a sweep. // 이전 scan에서 생성했던 포인트 클라우드들을 새로운 scan을 위해 초기화한다.
  if (true/*newSweep*/) {
    _sweepStart = scanTime;

    // clear cloud buffers   
    // _pixelCloud.clear();  // 새로 추가.

    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();

    // clear scan indices vector
    _scanIndices.clear();
  }
}


void BasicScanRegistration::updateIMUData(Vector3& acc, IMUState& newState)
{
  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time.        // 시간에 따라 IMU의 '위치'와 '속도'를 축척한다.
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);

    const IMUState& prevState = _imuHistory.last();           // 직전 imu state를 불러온다.

    float timeDiff = toSec(newState.stamp - prevState.stamp); // 직전 state와의 time difference를 계산.

    newState.position = prevState.position  // 현재 state의 위치 = 이전 state의 위치
                        + (prevState.velocity * timeDiff)     // + (이전 state의 속도 * 이전 state로 부터 흐른 시간)
                        + (0.5 * acc * timeDiff * timeDiff);  // + 변위 = (1/2*a*t^2) = 가속도(a)를 시간(t)에 대해 2번 적분한 것.

    newState.velocity = prevState.velocity  // 현재 state의 속도 = 이전 state의 속도
                        + acc * timeDiff;   // + 속도변화 = 가속도 * 시간.
  }

  _imuHistory.push(newState);   // _imuHistory에 저장한다. Cloud registration을 수행할 때 필요한 IMU state는 이 buffer를 이용해 접근한다.
}


void BasicScanRegistration::projectPointToStartOfSweep(pcl::PointXYZRGBNormal& point, float relTime)
{
  // project point to the start of the sweep using corresponding IMU data. 
  // 대응하는 IMU 데이터를 이용해, sweep의 시작 시점으로 각 포인트들을 투영한다.
  if (hasIMUData())
  {
    setIMUTransformFor(relTime);  // relTime(현재 시점)에 대해 "_imuCur(보간을 이용해 얻어진 현재 IMU의 position)"와 "_imuPositionShift(위치이동정보 = position shift)"를 생성해주는 메소드.
    transformToStartIMU(point);   // 위 메소드로 생성된 _imuCur와 _imuPositionShift를 이용해 입력한 포인트를 sweep의 시작 지점으로 투영해준다.
  }
}


void BasicScanRegistration::setIMUTransformFor(const float& relTime)
{
  interpolateIMUStateFor(relTime, _imuCur);         // 입력한 시간(relTime)에 대해 IMU state를 보간(interpolate)하는 메소드. 그 result가 _imuCur변수에 새롭게 저장됨.

//*
//std::cout << "imuCur: " << _imuCur.position << " " << _imuCur.velocity << std::endl;
//*
  float relSweepTime = toSec(_scanTime - _sweepStart) + relTime;                                  // relative sweep time.
  _imuPositionShift = _imuCur.position - _imuStart.position - _imuStart.velocity * relSweepTime;  // 누적된 IMU 위치와 보간된 IMU 위치의 차이. 즉 얼마만큼 이동되었는지.
}


void BasicScanRegistration::transformToStartIMU(pcl::PointXYZRGBNormal& point)  // 위 메소드로 생성된 _imuCur와 _imuPositionShift를 이용해 입력한 포인트를 sweep의 시작 지점으로 투영해준다.
{
  // rotate point to global IMU system.                                     // _imuCur를 이용해 global system으로 변환.
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);

  // add global IMU position shift                                          // position shift(_imuPositionShift)를 적용.
  point.x += _imuPositionShift.x();
  point.y += _imuPositionShift.y();
  point.z += _imuPositionShift.z();

  // rotate point back to local IMU system relative to the start IMU state. // 다시 start IMU state와 연관된 local system으로 변환.
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);
}


void BasicScanRegistration::interpolateIMUStateFor(const float &relTime, IMUState &outputState)
{
  double timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  while (_imuIdx < _imuHistory.size() - 1 && timeDiff > 0) {
    _imuIdx++;
    timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  }

  if (_imuIdx == 0 || timeDiff > 0) {
    outputState = _imuHistory[_imuIdx];
  } else {
    float ratio = -timeDiff / toSec(_imuHistory[_imuIdx].stamp - _imuHistory[_imuIdx - 1].stamp);
    IMUState::interpolate(_imuHistory[_imuIdx], _imuHistory[_imuIdx - 1], ratio, outputState);
  }
}


void BasicScanRegistration::extractFeatures(const uint16_t& beginIdx) // 현재의 포인트 클라우드로 부터 특징점을 추출하는 메소드.
{
  // extract features from individual scans.                          // 각 scan당 특징점들을 추출한다.
  size_t nScans = _scanIndices.size();

  for (size_t i = beginIdx; i < nScans; i++) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    size_t scanStartIdx = _scanIndices[i].first;
    size_t scanEndIdx = _scanIndices[i].second;

    // skip empty scans.                                              // 빈 scan은 건너뛴다.
    if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion) {
      continue;
    }

    // Quick&Dirty fix for relative point time calculation without IMU data
    /*float scanSize = scanEndIdx - scanStartIdx + 1;
    for (int j = scanStartIdx; j <= scanEndIdx; j++) {
      _laserCloud[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize;
    }*/

    // reset scan buffers.      // scan 버퍼를 리셋한다.
    setScanBuffersFor(scanStartIdx, scanEndIdx);

    // extract features from equally sized scan regions
    // 동일한 크기의 scan된 영역에 대해 특징점을 추출한다.
    for (int j = 0; j < _config.nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - j)
                   + (scanEndIdx - _config.curvatureRegion) * j) / _config.nFeatureRegions;
      size_t ep = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - 1 - j)
                   + (scanEndIdx - _config.curvatureRegion) * (j + 1)) / _config.nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;

      // reset region buffers
      setRegionBuffersFor(sp, ep);


      // extract corner features.   // corner 특징점 추출. _cornerPointsSharp와 _cornerPointsLessSharp를 push_back으로 생성한다.
      int largestPickedNum = 0;
      for (size_t k = regionSize; k > 0 && largestPickedNum < _config.maxCornerLessSharp;) {
        size_t idx = _regionSortIndices[--k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] > _config.surfaceCurvatureThreshold) {

          largestPickedNum++;
          if (largestPickedNum <= _config.maxCornerSharp) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            _cornerPointsSharp.push_back(_laserCloud[idx]);
          } else {
            _regionLabel[regionIdx] = CORNER_LESS_SHARP;
          }
          _cornerPointsLessSharp.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx);
        }
      }

      // extract flat surface features.   // flat 특징점 추출. _surfacePointsFlat를 push_back으로 생성한다.
      int smallestPickedNum = 0;
      for (int k = 0; k < regionSize && smallestPickedNum < _config.maxSurfaceFlat; k++) {
        size_t idx = _regionSortIndices[k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold) {

          smallestPickedNum++;
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx);
        }
      }

      // extract less flat surface features.   // less flat 특징점 추출. down size 되기 전인 _surfacePointsFlat인 surfPointsLessFlatScan를 push_back으로 생성한다.
      for (int k = 0; k < regionSize; k++) {
        if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
          surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
        }
      }
    }

    // down size less flat surface point cloud of current scan.   // 현 scan의 less flat 특징점들을 down size하여 _surfacePointsFlat을 생성한다.
    pcl::PointCloud<pcl::PointXYZRGBNormal> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(_config.lessFlatFilterSize, _config.lessFlatFilterSize, _config.lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    _surfacePointsLessFlat += surfPointsLessFlatScanDS;
  }
}



void BasicScanRegistration::updateIMUTransform()
{
  // _imuStart에 대해 입력.
  _imuTrans[0].x = _imuStart.pitch.rad();
  _imuTrans[0].y = _imuStart.yaw.rad();
  _imuTrans[0].z = _imuStart.roll.rad();

  // _imuCur에 대해 입력.
  _imuTrans[1].x = _imuCur.pitch.rad();
  _imuTrans[1].y = _imuCur.yaw.rad();
  _imuTrans[1].z = _imuCur.roll.rad();

  Vector3 imuShiftFromStart = _imuPositionShift;  // 누적된 IMU 위치와 보간된 IMU 위치의 차이. 즉 얼마만큼 이동되었는지.
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);  // imuShiftFromStart를 start IMU state와 연관된 local system으로 변환.

  // 0행과 1행간의 변위에 대해 입력.
  _imuTrans[2].x = imuShiftFromStart.x();
  _imuTrans[2].y = imuShiftFromStart.y();
  _imuTrans[2].z = imuShiftFromStart.z();

  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity; // 누적된 IMU 속도와 보간된 IMU 속도의 차이. 즉 현재까지의 속도변화.
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll); // imuVelocityFromStart를 start IMU state와 연관된 local system으로 변환.

  // 0행과 1행간의 속도변화에 대해 입력.
  _imuTrans[3].x = imuVelocityFromStart.x();
  _imuTrans[3].y = imuVelocityFromStart.y();
  _imuTrans[3].z = imuVelocityFromStart.z();
}


void BasicScanRegistration::setRegionBuffersFor(const size_t& startIdx, const size_t& endIdx)
{
  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize);
  _regionSortIndices.resize(regionSize);
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT);

  // calculate point curvatures and reset sort indices
  float pointWeight = -2 * _config.curvatureRegion;

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;

    for (int j = 1; j <= _config.curvatureRegion; j++) {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    _regionSortIndices[regionIdx] = i;
  }

  // sort point curvatures
  for (size_t i = 1; i < regionSize; i++) {
    for (size_t j = i; j >= 1; j--) {
      if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
      }
    }
  }
}


void BasicScanRegistration::setScanBuffersFor(const size_t& startIdx, const size_t& endIdx)
{
  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;
  _scanNeighborPicked.assign(scanSize, 0);

  // mark unreliable points as picked
  for (size_t i = startIdx + _config.curvatureRegion; i < endIdx - _config.curvatureRegion; i++) {
    const pcl::PointXYZRGBNormal& previousPoint = (_laserCloud[i - 1]);
    const pcl::PointXYZRGBNormal& point = (_laserCloud[i]);
    const pcl::PointXYZRGBNormal& nextPoint = (_laserCloud[i + 1]);

    float diffNext = calcSquaredDiff(nextPoint, point);

    if (diffNext > 0.1) {
      float depth1 = calcPointDistance(point);
      float depth2 = calcPointDistance(nextPoint);

      if (depth1 > depth2) {
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx - _config.curvatureRegion], _config.curvatureRegion + 1, 1);

          continue;
        }
      } else {
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx + 1], _config.curvatureRegion + 1, 1);
        }
      }
    }

    float diffPrevious = calcSquaredDiff(point, previousPoint);
    float dis = calcSquaredPointDistance(point);

    if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
      _scanNeighborPicked[i - startIdx] = 1;
    }
  }
}



void BasicScanRegistration::markAsPicked(const size_t& cloudIdx, const size_t& scanIdx)
{
  _scanNeighborPicked[scanIdx] = 1;

  for (int i = 1; i <= _config.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx + i] = 1;
  }

  for (int i = 1; i <= _config.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx - i] = 1;
  }
}


}
