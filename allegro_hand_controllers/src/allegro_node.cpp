// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "allegro_node.h"
#include "allegro_hand_driver/controlAllegroHand.h"


std::string jointNames[DOF_JOINTS] =
        {
                "index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
                "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
                "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
                "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3"
        };


AllegroNode::AllegroNode() {
  mutex = new boost::mutex();

  // Create arrays 16 long for each of the four joint state components
  msgJoint.position.resize(DOF_JOINTS);
  msgJoint.velocity.resize(DOF_JOINTS);
  msgJoint.effort.resize(DOF_JOINTS);
  msgJoint.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    msgJoint.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
  }

  // Get Allegro Hand information from parameter server
  // This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package
  std::string robot_name, manufacturer, origin, serial;
  double version;
  ros::param::get("~hand_info/robot_name", robot_name);
  ros::param::get("~hand_info/which_hand", whichHand);
  ros::param::get("~hand_info/manufacturer", manufacturer);
  ros::param::get("~hand_info/origin", origin);
  ros::param::get("~hand_info/serial", serial);
  ros::param::get("~hand_info/version", version);

  // Initialize CAN device
  canDevice = new controlAllegroHand();
  canDevice->init();
  usleep(3000);
  updateWriteReadCAN();

  // Start ROS time
  tstart = ros::Time::now();

  // Advertise joint states.
  joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
}

AllegroNode::~AllegroNode() {
  delete canDevice;
  delete mutex;
  nh.shutdown();
}

void AllegroNode::publishData() {
  // current position, velocity and effort (torque) published
  msgJoint.header.stamp = tnow;
  for (int i = 0; i < DOF_JOINTS; i++) {
    msgJoint.position[i] = current_position_filtered[i];
    msgJoint.velocity[i] = current_velocity_filtered[i];
    msgJoint.effort[i] = desired_torque[i];
  }
  joint_state_pub.publish(msgJoint);
}

void AllegroNode::updateWriteReadCAN() {
  // CAN bus communication.
  canDevice->setTorque(desired_torque);
  lEmergencyStop = canDevice->Update();
  canDevice->getJointInfo(current_position);

  if (lEmergencyStop < 0) {
    // Stop program when Allegro Hand is switched off
    ROS_ERROR("Allegro Hand Node is Shutting Down! (Emergency Stop)");
    ros::shutdown();
  }
}

void AllegroNode::updateController() {
  // Calculate loop time;
  tnow = ros::Time::now();
  dt = 1e-9 * (tnow - tstart).nsec;
  tstart = tnow;

  // save last iteration info
  for (int i = 0; i < DOF_JOINTS; i++) {
    previous_position[i] = current_position[i];
    previous_position_filtered[i] = current_position_filtered[i];
    previous_velocity[i] = current_velocity[i];
  }

  updateWriteReadCAN();

  // Low-pass filtering.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_position_filtered[i] = (0.6 * current_position_filtered[i]) +
                                   (0.198 * previous_position[i]) +
                                   (0.198 * current_position[i]);
    current_velocity[i] =
            (current_position_filtered[i] - previous_position_filtered[i]) / dt;
    current_velocity_filtered[i] = (0.6 * current_velocity_filtered[i]) +
                                   (0.198 * previous_velocity[i]) +
                                   (0.198 * current_velocity[i]);
    current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
  }

  computeDesiredTorque();

  publishData();

  frame++;
}


// Interrupt-based control is not recommended by SimLab. I have not tested it.
void AllegroNode::timerCallback(const ros::TimerEvent& event)
{
  updateController();
}

ros::Timer AllegroNode::startTimerCallback() {
  ros::Timer timer = nh.createTimer(ros::Duration(0.001),
                                    &AllegroNode::timerCallback, this);
  return timer;
}
