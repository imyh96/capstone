#pragma once

#include <utility>
#include <vector>

#include <pcl/point_cloud.h>

#include "Angle.h"
#include "Vector3.h"
#include "CircularBuffer.h"
#include "time_utils.h"

#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <cmath>


namespace loam
{



  /** \brief A pair describing the start end end index of a range. */ // 스캔한 범위의 처음과 끝의 index를 나타내주는 pair.
  typedef std::pair<size_t, size_t> IndexRange;



  /** Point label options. */
  // 포인트의 type을 나타내기 위한 define 값들
  enum PointLabel
  {
    CORNER_SHARP = 2,       ///< sharp corner point
    CORNER_LESS_SHARP = 1,  ///< less sharp corner point
    SURFACE_LESS_FLAT = 0,  ///< less flat surface point
    SURFACE_FLAT = -1       ///< flat surface point
  };


  /** Scan Registration configuration parameters. */
  // 스캔 등록을 위한 매개변수들의 구성을 위한 클래스
  class RegistrationParams
  {
  public:
    RegistrationParams(const float& scanPeriod_ = 0.1, //0.1
      const int& imuHistorySize_ = 200,
      const int& nFeatureRegions_ = 6,
      const int& curvatureRegion_ = 5,
      const int& maxCornerSharp_ = 2,
      const int& maxSurfaceFlat_ = 4,
      const float& lessFlatFilterSize_ = 0.2,
      const float& surfaceCurvatureThreshold_ = 0.1);

    /** The time per scan. */ // 한 스캔당 시간.
    float scanPeriod;

    /** The size of the IMU history state buffer. */ // imu의 history state 버퍼 사이즈
    int imuHistorySize;

    /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */ // 한번의 스캔에서 특징점을 배포(distribute)할 때 사용되는 동일한 크기의 (특징)영역들의 갯수
    int nFeatureRegions;

    /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */ // 한 포인트의 곡률을 계산하는데 사용되는 주위 포인트들의 갯수
    int curvatureRegion;

    /** The maximum number of sharp corner points per feature region. */ // 특징영역별 최대 sharp corner 포인트들의 갯수
    int maxCornerSharp;

    /** The maximum number of less sharp corner points per feature region. */ // 특징영역별 최대 less sharp corner 포인트들의 갯수
    int maxCornerLessSharp;

    /** The maximum number of flat surface points per feature region. */ // 특징영역별 최대 flat surface 포인트들의 갯수
    int maxSurfaceFlat;

    /** The voxel size used for down sizing the remaining less flat surface points. */ // 남은 less flat surface 포인트들을 축소(down sizing)하는데 사용되는 복셀의 크기
    float lessFlatFilterSize;

    /** The curvature threshold below / above a point is considered a flat / corner point. */ // flat/corner 특징점들을 분별할 때 사용되는 곡률 threshold 값.
    float surfaceCurvatureThreshold;
  };



  /** IMU state data. */
  // IMU센서의 상태 데이터
  typedef struct IMUState
  {
    /** The time of the measurement leading to this state (in seconds). */ // 현재 상태에 이르기까지 경과한 시간.
    Time stamp;

    /** The current roll angle. */ // 현재 roll 각도
    Angle roll;

    /** The current pitch angle. */ // 현재 pitch 각도
    Angle pitch;

    /** The current yaw angle. */ // 현재 yaw 각도
    Angle yaw;

    /** The accumulated global IMU position in 3D space. */ // 축척된 3차원 world 좌표계에서의 IMU의 위치.
    Vector3 position;

    /** The accumulated global IMU velocity in 3D space. */ // 축척된 3차원 world 좌표계에서의 IMU의 속도.
    Vector3 velocity;

    /** The current (local) IMU acceleration in 3D space. */ // 현재 IMU의 3차원상에서의 가속도.
    Vector3 acceleration;



    /** \brief Interpolate between two IMU states.
    *
    * 두 IMU 상태 값 사이의 보간(Interpolate)을 위한 메소드.
    * 
    * @param start the first IMUState // 첫번째 imu의 상태정보
    * @param end the second IMUState // 두번째 imu의 상태정보
    * @param ratio the interpolation ratio  보간 비율.
    * @param result the target IMUState for storing the interpolation result  보간된 결과를 보관하기 위한 IMUState 객체.
    */
    static void interpolate(const IMUState& start,
      const IMUState& end,
      const float& ratio,
      IMUState& result)
    {
      float invRatio = 1 - ratio;

      result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
      result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
      if (start.yaw.rad() - end.yaw.rad() > M_PI)
      {
        result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() + 2 * M_PI) * ratio;
      }
      else if (start.yaw.rad() - end.yaw.rad() < -M_PI)
      {
        result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() - 2 * M_PI) * ratio;
      }
      else
      {
        result.yaw = start.yaw.rad() * invRatio + end.yaw.rad() * ratio;
      }

      result.velocity = start.velocity * invRatio + end.velocity * ratio;
      result.position = start.position * invRatio + end.position * ratio;
    };


  } IMUState;



  // scanRegistration 클래스에서 기본적으로 사용되는 멤버 변수, 메소드들을 정의해 둔 클래스.
  class BasicScanRegistration
  {
  public:
    /** \brief Process a new cloud as a set of scanlines. // 하나의 새 포인트 클라우드를 scanline들(layer들)의 집합으로써 처리하는 메소드.
    *
    * @param scanTime the time relative to the scan time   // 현재 스캔이 수행되는 시간.
    */
    void processScanlines(const Time& scanTime, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> const& laserCloudScans);


    bool configure(const RegistrationParams& config = RegistrationParams()); 


    /** \brief Update new IMU state. NOTE: MUTATES ARGS! */ // 새 imu state로 업데이트 해주는 메소드.
    void updateIMUData(Vector3& acc, IMUState& newState);



    /** \brief Project a point to the start of the sweep using corresponding IMU data
    * 대응하는 IMU 데이터를 이용해, sweep의 시작 시점으로 포인트를 투영한다.
    * 
    * @param point The point to modify.
    * @param relTime The time to project by.   // 포인트를 투영할 시점.
    */
    void projectPointToStartOfSweep(pcl::PointXYZRGBNormal& point, float relTime);


    // 각각의 값들을 return 해주는 메소드.
    auto const& imuTransform          () { return _imuTrans             ; }
    auto const& sweepStart            () { return _sweepStart           ; }

    // auto & pixelCloud            () { return _pixelCloud          ; }

    auto & laserCloud            () { return _laserCloud           ; }
    auto const& cornerPointsSharp     () { return _cornerPointsSharp    ; }
    auto const& cornerPointsLessSharp () { return _cornerPointsLessSharp; }
    auto const& surfacePointsFlat     () { return _surfacePointsFlat    ; }
    auto const& surfacePointsLessFlat () { return _surfacePointsLessFlat; }
    auto const& config                () { return _config               ; }

    // cv::Mat _mat_left;
    // cv::Mat _mat_right;
    cv::Mat _mat_depth;

    // 라이다 depth이미지 띄우기
    //cv::Mat _mat_lidar_depth = cv::Mat::zeros(720, 1280, CV_8UC3);
    // cv::Mat _mat_lidar_depth = cv::Mat::zeros(720, 1280, CV_32FC1);

    //HD 내부파라미터
    cv::Mat K;
    //  = (cv::Mat_<float>(3,3) <<  528.82, 0, 639.07,
    //                                       0, 528.49, 353.9245,
    //                                       0, 0, 1);

    // cv::Mat K = (cv::Mat_<float>(3,3) <<  0.52882, 0, 639.07, 
    //                                       0, 0.52849, 353.9245,
    //                                       0, 0, 1);
    
    cv::Mat E = (cv::Mat_<float>(3,4) <<  -1,  0, 0, 0.165, //0.06 ,0.15, 0.165
                                           0, -1, 0, 0.066, //-0.056, -0.026, 0.066
                                           0,  0, 1, 0.0444); //0.0444
    cv::Mat KE;
    cv::Mat pseu_inv_KE;

    float* depths;

    bool picture = false;

  private:
    /** \brief Check if IMU data is available. */ // IMU 데이터가 존재하는지 확인하는 bool 메소드.
    inline bool hasIMUData() { return _imuHistory.size() > 0; };


    /** \brief Set up the current IMU transformation for the specified relative time. // 입력한 relative time에 대해 현재 IMU transformation을 세팅하는 메소드.
     * 
     * @param relTime the time relative to the scan time
     */
    void setIMUTransformFor(const float& relTime);


    /** \brief Project the given point to the start of the sweep, using the current IMU state and position shift.
     * 현재 IMU state와 위치이동정보(position shift)를 이용해, 입력한 포인트를 sweep의 시작 지점으로 투영하는 메소드
     *
     * @param point the point to project. // 투영할 포인트
     */
    void transformToStartIMU(pcl::PointXYZRGBNormal& point);


    /** \brief Prepare for next scan / sweep. // 다음 scan/sweep을 위한 준비(리셋)를 해 주는 메소드.
     *
     * @param scanTime the current scan time
     * @param newSweep indicator if a new sweep has started
     */
    void reset(const Time& scanTime);


    /** \brief Extract features from current laser cloud. // 현재의 포인트 클라우드로 부터 특징점을 추출하는 메소드.
     *
     * @param beginIdx the index of the first scan to extract features from // 특징점을 추출할 첫번째 scan을 나타내는 index값.
     */
    void extractFeatures(const uint16_t& beginIdx = 0);


    /** \brief Set up region buffers for the specified point range. // 특정 포인트 범위를 위한 영역 버퍼를 세팅하는 메소드.
     *
     * @param startIdx the region start index
     * @param endIdx the region end index
     */
    void setRegionBuffersFor(const size_t& startIdx, const size_t& endIdx);


    /** \brief Set up scan buffers for the specified point range. // 특정 포인트 범위를 위한 스캔 버퍼를 세팅하는 메소드.
     *
     * @param startIdx the scan start index
     * @param endIdx the scan start index
     */
    void setScanBuffersFor(const size_t& startIdx, const size_t& endIdx);


    /** \brief Mark a point and its neighbors as picked.  // 포인트 하나와 그 이웃 포인트들을 picked로 마킹하는 메소드.
     *
     * This method will mark neighboring points within the curvature region as picked,
     * as long as they remain within close distance to each other.
     *
     * @param cloudIdx the index of the picked point in the full resolution cloud
     * @param scanIdx the index of the picked point relative to the current scan
     */
    void markAsPicked(const size_t& cloudIdx, const size_t& scanIdx);


    /** \brief Try to interpolate the IMU state for the given time. // 입력한 시간에 대해 IMU state를 보간(interpolate)하는 메소드.
     *
     * @param relTime the time relative to the scan time  // 입력하는 시간.
     * @param outputState the output state instance.      // 이 메소드의 출력 result가 저장될 객체.
     */
    void interpolateIMUStateFor(const float& relTime, IMUState& outputState);


    void updateIMUTransform();  // imu의 transform을 업데이트 하는 메소드.


  private:
    RegistrationParams _config;  ///< registration parameter // 등록을 위한 클래스의 객체 생성.

    pcl::PointCloud<pcl::PointXYZRGBNormal> _laserCloud;   ///< full resolution input cloud
    // pcl::PointCloud<pcl::PointXYZRGB> _pixelCloud;

    std::vector<IndexRange> _scanIndices;          ///< start and end indices of the individual scans withing the full resolution cloud // 전체 해상도 포인트 클라우드의 각 스캔 범위의 처음과 끝을 나타내는 인덱스들의 벡터.


    // 추출된 각 타입의 특징점들로 이루어진 포인트 클라우드 4종류.
    pcl::PointCloud<pcl::PointXYZRGBNormal> _cornerPointsSharp;      ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal> _cornerPointsLessSharp;  ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal> _surfacePointsFlat;      ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal> _surfacePointsLessFlat;  ///< less flat surface points cloud

    Time _sweepStart;            ///< time stamp of beginning of current sweep                                                                      // 현재(current) sweep이 시작했을 때의 시간.
    Time _scanTime;              ///< time stamp of most recent scan                                                                                // 가장 최근에 scan이 시행된 시간.
    IMUState _imuStart;                     ///< the interpolated IMU state corresponding to the start time of the currently processed laser scan   // 현재 처리 된 레이저 스캔의 시작 시간에 해당하는 보간 된 IMU 상태
    IMUState _imuCur;                       ///< the interpolated IMU state corresponding to the time of the currently processed laser scan point   // 현재 처리 된 레이저 스캔 포인트의 시간에 해당하는 보간 된 IMU 상태
    Vector3 _imuPositionShift;              ///< position shift between accumulated IMU position and interpolated IMU position                      // 누적된 IMU 위치와 보간된 IMU 위치의 차이. 즉 얼만큼 이동되었는지.
    size_t _imuIdx = 0;                         ///< the current index in the IMU history                                                           // IMU 기록(history)의 현재 index값.
    CircularBuffer<IMUState> _imuHistory;   ///< history of IMU states for cloud registration // 포인트 클라우드 registration에 사용하기 위해 buffer에 imu state들을 기록해둔다. Cloud registration을 수행할 때 필요한 IMU state는 이 buffer를 이용해 접근한다.


    pcl::PointCloud<pcl::PointXYZ> _imuTrans = { 4,1 };  ///< IMU transformation information        // IMU 센서 데이터로부터 얻어지는 transformation(변환행렬)을 담고있는 변수.


    std::vector<float> _regionCurvature;      ///< point curvature buffer                           // 각 포인트의 곡률에 대한 버퍼.
    std::vector<PointLabel> _regionLabel;     ///< point label buffer                               // 각 포인트의 label에 대한 버퍼.
    std::vector<size_t> _regionSortIndices;   ///< sorted region indices based on point curvature   // 포인트의 곡률에 기반해 정렬된 영역의 인덱스
    std::vector<int> _scanNeighborPicked;     ///< flag if neighboring point was already picked     // 포인트와 이웃한 포인트가 이미 pick되었는지를 나타내는 flag.

    //////////////////////////////////////////
    
  };

}

