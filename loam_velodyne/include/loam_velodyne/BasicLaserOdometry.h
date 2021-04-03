#pragma once
#include "Twist.h"
#include "nanoflann_pcl.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <queue>
#include <set>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>


namespace loam
{

  /** \brief Implementation of the LOAM laser odometry component.
   * 롬에서의 레이저 주행거리 측정 컴포넌트의 구현을 위한 기본 클래스.
   */
  class BasicLaserOdometry
  {
  public:
    explicit BasicLaserOdometry(float scanPeriod = 0.1, size_t maxIterations = 25);


    /** \brief Try to process buffered data. */ // subscribe하여 저장된 데이터들을 처리(process)한다.
    void process();


    // IMU의 변환행렬을 업데이트 해 주는 메소드. 
    // scan registration 노드에서 하나의 변수로 보내진 정보들을 4종류의 변수에 나누어 저장한다.
    void updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans);


    // 각 포인트 클라우드 데이터, 변환을 리턴해주는 get메소드.
    auto& cornerPointsSharp()     { return _cornerPointsSharp; }
    auto& cornerPointsLessSharp() { return _cornerPointsLessSharp; }
    auto& surfPointsFlat()        { return _surfPointsFlat; }
    auto& surfPointsLessFlat()    { return _surfPointsLessFlat; }
    auto& laserCloud() { return _laserCloud; }

    auto const& transformSum() { return _transformSum; }
    auto const& transform()    { return _transform;    }
    auto const& lastCornerCloud () { return _lastCornerCloud ; }
    auto const& lastSurfaceCloud() { return _lastSurfaceCloud; }


    // 기본 parameter들의 setting용 메소드들.
    void setScanPeriod(float val)     { _scanPeriod    = val; }
    void setMaxIterations(size_t val) { _maxIterations = val; }
    void setDeltaTAbort(float val)    { _deltaTAbort = val;   }
    void setDeltaRAbort(float val)    { _deltaRAbort = val;   }

    // 기본 parameter값을 리턴해 주는 get 메소드들.
    auto frameCount()    const { return _frameCount;    }
    auto scanPeriod()    const { return _scanPeriod;    }
    auto maxIterations() const { return _maxIterations; }
    auto deltaTAbort()   const { return _deltaTAbort;   }
    auto deltaRAbort()   const { return _deltaRAbort;   }


    /** \brief Transform the given point cloud to the end of the sweep. // 주어진 포인트 클라우드를 sweep의 끝 지점으로 변환(transform)해 주는 메소드.
     *  
     * @param cloud the point cloud to transform.                       // 변환할 포인트 클라우드.
     */
    size_t transformToEnd(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud);


  private:
    /** \brief Transform the given point to the start of the sweep. // 주어진 포인트 클라우드를 sweep의 시작 지점으로 변환(transform)해 주는 메소드.
     *
     * @param pi the point to transform.                            // 변환할 포인트 클라우드.
     * @param po the point instance for storing the result.         // 메소드의 리턴값을 저장하기 위한 포인트 클라우드 객체.
     */
    void transformToStart(const pcl::PointXYZRGBNormal& pi, pcl::PointXYZRGBNormal& po);


    void pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                           const Angle& blx, const Angle& bly, const Angle& blz,
                           const Angle& alx, const Angle& aly, const Angle& alz,
                           Angle &acx, Angle &acy, Angle &acz);

    void accumulateRotation(Angle cx, Angle cy, Angle cz,
                            Angle lx, Angle ly, Angle lz,
                            Angle &ox, Angle &oy, Angle &oz);
    

  private:
    float _scanPeriod;       ///< time per scan.                // 스캔 주기
    long _frameCount;        ///< number of processed frames.   // 프레임 수
    size_t _maxIterations;   ///< maximum number of iterations. // 최대 반복 횟수
    
    bool _systemInited;      ///< initialization flag.          // 초기화되었는지 체크

    float _deltaTAbort;     ///< optimization abort threshold for deltaT.  // 이동 threshold?
    float _deltaRAbort;     ///< optimization abort threshold for deltaR.  // 회전 threshold?


    // multi-scan registration 노드로 부터 새 특징점 클라우드를 넘겨받아, _Points변수가 덮어씌워지므로, motion estimation을 위해 이전 sweep의 특징점 클라우드를 저장해 두기 위한 변수들.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _lastCornerCloud;    ///< last corner points cloud.    // 이전 corner 클라우드.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _lastSurfaceCloud;   ///< last surface points cloud.   // 이전 surface 클라우드.

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloudOri;      ///< point selection              // 특징점들의 neighbor로 골라진 포인트들.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _coeffSel;           ///< point selection coefficientsw

    nanoflann::KdTreeFLANN<pcl::PointXYZRGBNormal> _lastCornerKDTree;   ///< last corner cloud KD-tree  // 빠른 indexing을 위한 KD-tree
    nanoflann::KdTreeFLANN<pcl::PointXYZRGBNormal> _lastSurfaceKDTree;  ///< last surface cloud KD-tree

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cornerPointsSharp;      ///< sharp corner points cloud        // 4개의 특징점 클라우드와 1개의 전체 해상도 포인트 클라우드를 담을 변수.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cornerPointsLessSharp;  ///< less sharp corner points cloud   // scan registration 노드로 부터 subscribe해 온것을 후처리할 때 사용.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _surfPointsFlat;         ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _surfPointsLessFlat;     ///< less flat surface points cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _laserCloud;             ///< full resolution cloud

    std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer
    std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer

    std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer
    std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer
    std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer

    Twist _transform;     ///< optimized pose transformation             // 최적화된 pose 변환행렬.
    Twist _transformSum;  ///< accumulated optimized pose transformation // 최적화된 pose 변환행렬의 축척.


    // updateIMU 함수에 의해 subscribe해 온 imu 변환행렬의 값으로 업데이트 되는 변수들. 
    Angle _imuRollStart, _imuPitchStart, _imuYawStart;    // 시작시의 roll pitch yaw.
    Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;          // 종료시의 roll pitch yaw.
    Vector3 _imuShiftFromStart;                           // 시작과 종료 사이의 위치 변화.
    Vector3 _imuVeloFromStart;                            // 시작과 종료 사이의 속도 변화.

  };

} // end namespace loam
