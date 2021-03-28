#include <ros/ros.h>
#include "loam_velodyne/LaserOdometry.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserOdometry laserOdom(0.1);  //0.1은 스캔 주기
  
  // node는 publish나 subscribe에 사용하고,
  // private는 파라미터 서버에 접근할때, 사용
  // 예를 들어 launch파일을 통해서 넘겨준 파라미터에 접근할때
  // 일반 노드 핸들은 파라미터의 이름에 노드의 네임 스페이스를 붙여줘야함 // 노드의 이름을 바꿔주면 무용지물 
  // 반면 private는 파라미터 이름만으로 접근 가능
  
  if (laserOdom.setup(node, privateNode)) {
    // initialization successful
    laserOdom.spin();
  }

  return 0;
}
