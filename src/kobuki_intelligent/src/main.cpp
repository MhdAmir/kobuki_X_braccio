#include <memory>
#include <ros/ros.h>
#include "kobuki_intelligent/intelligent.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "Intelligent");

  std::unique_ptr<Intelligent> intelligent(new Intelligent());

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    intelligent->Loop();
    rate.sleep();
  }

  return 0;
}