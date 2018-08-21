
#include <ros/ros.h>
#include <argus-cam.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gscam_publisher");
  ros::NodeHandle nh, nh_private("~");

  gscam::GSCam gscam_driver(nh, nh_private);
  gscam_driver.run();

  return 0;
}

