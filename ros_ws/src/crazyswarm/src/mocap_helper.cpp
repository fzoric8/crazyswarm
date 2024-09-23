#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_helper");

  ros::NodeHandle nl("~");

  ros::Publisher uavPosePub = nl.advertise<geometry_msgs::PoseStamped>("opt_elios_pose", 1);
  ros::Publisher eePosePub = nl.advertise<geometry_msgs::PoseStamped>("opt_ee_pose", 1);
  ros::Publisher bbp1PosePub = nl.advertise<geometry_msgs::PoseStamped>("opt_bbp1_pose", 1);
  ros::Publisher bbp2PosePub = nl.advertise<geometry_msgs::PoseStamped>("opt_bbp2_pose", 1);

  // get UAV and the ee name
  std::string uavName("elios3"); 
  std::string eeName("elios3_ee"); 
  std::string bbp1Name("bbp1");
  std::string bbp2Name("bbp2");

  std::string motionCaptureType;
  nl.param<std::string>("motion_capture_type", motionCaptureType, "vicon");

  std::map<std::string, std::string> cfg;
  std::string hostname;
  nl.getParam("motion_capture_host_name", hostname);
  cfg["hostname"] = hostname;
  if (nl.hasParam("motion_capture_interface_ip")) {
    std::string interface_ip;
    nl.param<std::string>("motion_capture_interface_ip", interface_ip);
    cfg["interface_ip"] = interface_ip;
  }

  std::unique_ptr<libmotioncapture::MotionCapture> mocap(libmotioncapture::MotionCapture::connect(motionCaptureType, cfg));
  if (!mocap) {
    throw std::runtime_error("Unknown motion capture type!");
  }

  for (size_t frameId = 0; ros::ok(); ++frameId) {
    std::cout << "frame " << frameId << ":" << std::endl;
    // Get a frame
    mocap->waitForNextFrame();
    const auto& markers = mocap->pointCloud();

    std::cout << "    points:" << std::endl;

    for (size_t i = 0; i < markers.rows(); ++i) {
      const auto& point = markers.row(i);
      std::cout << "      \"" << i << "\": [" << point(0) << "," << point(1) << "," << point(2) << "]" << std::endl;
    }

    if (mocap->supportsRigidBodyTracking()) {
      const auto& rigidBodies = mocap->rigidBodies();

      std::cout << "    rigidBodies:" << std::endl;

      for (auto const& kv: rigidBodies) {
        const auto& body = kv.second;
        std::cout << "      \"" << body.name() << "\":" << std::endl;

        Eigen::Vector3f position = body.position();
        Eigen::Quaternionf rotation = body.rotation();
        std::cout << "       position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
        std::cout << "       rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
                                          << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;
        
        std::string bodyName(body.name());

        if (bodyName.compare(uavName) == 0){ 
          geometry_msgs::PoseStamped uavPoseMsg;
          uavPoseMsg.header.stamp = ros::Time::now();
          uavPoseMsg.header.frame_id = "world";
          uavPoseMsg.pose.position.x = position(0);
          uavPoseMsg.pose.position.y = position(1);
          uavPoseMsg.pose.position.z = position(2);
          uavPoseMsg.pose.orientation.w = rotation.w();
          uavPoseMsg.pose.orientation.x = rotation.vec()(0);
          uavPoseMsg.pose.orientation.y = rotation.vec()(1);
          uavPoseMsg.pose.orientation.z = rotation.vec()(2);
          uavPosePub.publish(uavPoseMsg);
        }
        else if (bodyName.compare(eeName) == 0)
        {
          geometry_msgs::PoseStamped eePoseMsg;
          eePoseMsg.header.stamp = ros::Time::now();
          eePoseMsg.header.frame_id = "world";
          eePoseMsg.pose.position.x = position(0);
          eePoseMsg.pose.position.y = position(1);
          eePoseMsg.pose.position.z = position(2);
          eePoseMsg.pose.orientation.w = rotation.w();
          eePoseMsg.pose.orientation.x = rotation.vec()(0);
          eePoseMsg.pose.orientation.y = rotation.vec()(1);
          eePoseMsg.pose.orientation.z = rotation.vec()(2);
          eePosePub.publish(eePoseMsg);
        }
        else if (bodyName.compare(bbp1Name) == 0)
        {
          geometry_msgs::PoseStamped bbp1PoseMsg;
          bbp1PoseMsg.header.stamp = ros::Time::now();
          bbp1PoseMsg.header.frame_id = "world";
          bbp1PoseMsg.pose.position.x = position(0);
          bbp1PoseMsg.pose.position.y = position(1);
          bbp1PoseMsg.pose.position.z = position(2);
          bbp1PoseMsg.pose.orientation.w = rotation.w();
          bbp1PoseMsg.pose.orientation.x = rotation.vec()(0);
          bbp1PoseMsg.pose.orientation.y = rotation.vec()(1);
          bbp1PoseMsg.pose.orientation.z = rotation.vec()(2);
          bbp1PosePub.publish(bbp1PoseMsg);
        }
        else if (bodyName.compare(bbp2Name) == 0)
        {
          geometry_msgs::PoseStamped bbp2PoseMsg;
          bbp2PoseMsg.header.stamp = ros::Time::now();
          bbp2PoseMsg.header.frame_id = "world";
          bbp2PoseMsg.pose.position.x = position(0);
          bbp2PoseMsg.pose.position.y = position(1);
          bbp2PoseMsg.pose.position.z = position(2);
          bbp2PoseMsg.pose.orientation.w = rotation.w();
          bbp2PoseMsg.pose.orientation.x = rotation.vec()(0);
          bbp2PoseMsg.pose.orientation.y = rotation.vec()(1);
          bbp2PoseMsg.pose.orientation.z = rotation.vec()(2);
          bbp2PosePub.publish(bbp2PoseMsg);
        }
        else{
          std::cout << "I didn't found: " << bodyName << std::endl; 
        }
      }
    }


    ros::spinOnce();
  }

  return 0;
}
