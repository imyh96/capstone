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

////////////////////////////////////
#include <pcl/io/ply_io.h>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
////////////////////////////////////

#define PCNUM 2000000 // number of _laserCloudFullResColorStack point clouds
#define PLYFILENAME "/home/cgvlab/ply_test2/output_zedTrans134_thread_OV200.ply"

namespace loam
{

/** \brief Implementation of the LOAM transformation maintenance component. 변환(행렬) 유지관리(maintenance) 컴포넌트의 구현을 위한 클래스
 *
 */
class BasicTransformMaintenance
{
public:
   explicit BasicTransformMaintenance();

   void updateOdometry(double pitch, double yaw, double roll, double x, double y, double z);    // Odometry 업데이트용 메소드.

   void updateMappingTransform(Twist const& transformAftMapped, Twist const& transformBefMapped); // Mapping 변환행렬(Transform) 업데이트용 메소드-1.

   void updateMappingTransform(double pitch, double yaw, double roll,
      double x, double y, double z,
      double twist_rot_x, double twist_rot_y, double twist_rot_z,
      double twist_pos_x, double twist_pos_y, double twist_pos_z);    // Mapping 변환행렬(Transform) 업데이트용 메소드-2.


   void transformAssociateToMap();  

   /////////////////////////////////////////////////
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr _laserCloudMap;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr _laserCloudSurround;
   std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _laserCloudMapArray;

   bool _newLaserCloudMap = false;
   // bool _turnOffSignal = false;
   int SInd = 0;
   /////////////////////////////////////////////////
   
   
   // result accessor
   auto const& transformMapped() const { return _transformMapped; } // 결과(result) 접근용 get 메소드.

private:
   float _transformSum[6]{};
   float _transformIncre[6]{};
   float _transformMapped[6]{};
   float _transformBefMapped[6]{};
   float _transformAftMapped[6]{};
};

} // end namespace loam

