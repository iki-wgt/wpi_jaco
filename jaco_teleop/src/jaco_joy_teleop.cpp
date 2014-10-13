/*!
 * \jaco_joy_teleop.h
 * \brief Allows for control of the jaco arm with a joystick.
 *
 * jaco_joy_teleop creates a ROS node that allows for the control of the
 * JACO arm with a joystick. This node listens to a /joy topic.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date June 25, 2014
 */

#include <jaco_teleop/jaco_joy_teleop.h>

using namespace std;

jaco_joy_teleop::jaco_joy_teleop()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");

  // create the ROS topics
  cartesian_cmd = node.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &jaco_joy_teleop::joy_cback, this);

  // read in throttle values
  private_nh.param<double>("linear_throttle_factor", linear_throttle_factor, 1.0);
  private_nh.param<double>("angular_throttle_factor", angular_throttle_factor, 1.0);
  private_nh.param<double>("finger_throttle_factor", finger_throttle_factor, 1.0);
  string str;


  //initialize everything
  stopMessageSentArm = true;
  stopMessageSentFinger = true;
  EStopEnabled = false;
  helpDisplayed = false;
  mode = ARM_CONTROL;
  fingerCmd.position = false;
  fingerCmd.armCommand = false;
  fingerCmd.fingerCommand = true;
  fingerCmd.repeat = true;
  fingerCmd.fingers.resize(3);
  cartesianCmd.position = false;
  cartesianCmd.armCommand = true;
  cartesianCmd.fingerCommand = false;
  cartesianCmd.repeat = true;

  ROS_INFO("JACO joystick teleop started");


  calibrated = true;
}

void jaco_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // make sure triggers are calibrated before continuint if an analog controller was specified
  if (!calibrated)
  {
    if (!initLeftTrigger && joy->axes.at(2) == 1.0)
      initLeftTrigger = true;

    if (!initRightTrigger && joy->axes.at(5) == 1.0)
      initRightTrigger = true;

    if (initLeftTrigger && initRightTrigger)
    {
      calibrated = true;
      ROS_INFO("Controller calibration complete!");
    }

    return;
  }

  //software emergency stop

  if (joy->buttons.at(8) == 1)
    EStopEnabled = true;
  else if (joy->buttons.at(9) == 1)
    EStopEnabled = false;


  int buttonIndex;

  switch (mode)
  {
    case ARM_CONTROL:
      //careful these settings are for a jaco arm mounted upside down!!!

      // left joystick controls the cartesian movement in world space
      cartesianCmd.arm.linear.x = -joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor;
      cartesianCmd.arm.linear.y = -joy->axes.at(0) * MAX_TRANS_VEL * linear_throttle_factor;
      cartesianCmd.arm.linear.z = -joy->axes.at(2) * MAX_TRANS_VEL * linear_throttle_factor;


      //right joystick controls roll pitch yaw in gripper coordinate system with x pointing 
      //into finger direction, z pointing up (ignore angular.x,y,z)
      //pitch
      cartesianCmd.arm.angular.x = joy->axes.at(4) * MAX_ANG_VEL * angular_throttle_factor;
      //yaw
      cartesianCmd.arm.angular.y = joy->axes.at(5) * MAX_ANG_VEL * angular_throttle_factor;
      //roll
      cartesianCmd.arm.angular.z = joy->axes.at(3) * MAX_ANG_VEL * angular_throttle_factor;


      //mode switching

      buttonIndex = 0;


      if (joy->buttons.at(buttonIndex) == 1)
      {
        //cancel trajectory and switch to finger control mode
        cartesianCmd.arm.linear.x = 0.0;
        cartesianCmd.arm.linear.y = 0.0;
        cartesianCmd.arm.linear.z = 0.0;
        cartesianCmd.arm.angular.x = 0.0;
        cartesianCmd.arm.angular.y = 0.0;
        cartesianCmd.arm.angular.z = 0.0;
        cartesian_cmd.publish(cartesianCmd);
        mode = FINGER_CONTROL;

        ROS_INFO("Activated finger control mode");
      }
      break;
    case FINGER_CONTROL:

      //individual finger control

      //thumb controlled

      //fingerCmd.fingers[0] = -joy->axes.at(4) * MAX_FINGER_VEL * finger_throttle_factor;

      //top finger


      //bottom finger controlled


      //control full gripper (outprioritizes individual finger control)
      fingerCmd.fingers[0] = -joy->axes.at(1) * MAX_FINGER_VEL * finger_throttle_factor;
      fingerCmd.fingers[1] = fingerCmd.fingers[0];
      fingerCmd.fingers[2] = fingerCmd.fingers[0];
  

      //mode switching
      buttonIndex = 1;


      if (joy->buttons.at(buttonIndex) == 1)
      {
        //cancel trajectory and switch to arm control mode
        fingerCmd.fingers[0] = 0.0;
        fingerCmd.fingers[1] = 0.0;
        fingerCmd.fingers[2] = 0.0;
        cartesian_cmd.publish(fingerCmd);
        mode = ARM_CONTROL;

        ROS_INFO("Activated arm control mode");
      }
      break;
  }
}

void jaco_joy_teleop::publish_velocity()
{
  //publish stop commands if EStop is enabled
  if (EStopEnabled)
  {
    cartesianCmd.arm.linear.x = 0.0;
    cartesianCmd.arm.linear.y = 0.0;
    cartesianCmd.arm.linear.z = 0.0;
    cartesianCmd.arm.angular.x = 0.0;
    cartesianCmd.arm.angular.y = 0.0;
    cartesianCmd.arm.angular.z = 0.0;
    fingerCmd.fingers[0] = 0.0;
    fingerCmd.fingers[1] = 0.0;
    fingerCmd.fingers[2] = 0.0;

    cartesian_cmd.publish(cartesianCmd);
    cartesian_cmd.publish(fingerCmd);

    return;
  }

  switch (mode)
  {
    case ARM_CONTROL:
      //only publish stop message once; this allows other nodes to publish velocities
      //while the controller is not being used
      if (cartesianCmd.arm.linear.x == 0.0 && cartesianCmd.arm.linear.y == 0.0 && cartesianCmd.arm.linear.z == 0.0
          && cartesianCmd.arm.angular.x == 0.0 && cartesianCmd.arm.angular.y == 0.0
          && cartesianCmd.arm.angular.z == 0.0)
      {
        if (!stopMessageSentArm)
        {
          cartesian_cmd.publish(cartesianCmd);
          stopMessageSentArm = true;
        }
      }
      else
      {
        // send the twist command
        cartesian_cmd.publish(cartesianCmd);
        stopMessageSentArm = false;
      }
      break;
    case FINGER_CONTROL:
      //only publish stop message once; this allows other nodes to publish velocities
      //while the controller is not being used
      if (fingerCmd.fingers[0] == 0.0 && fingerCmd.fingers[1] == 0.0 && fingerCmd.fingers[2] == 0.0)
      {
        if (!stopMessageSentFinger)
        {
          cartesian_cmd.publish(fingerCmd);
          stopMessageSentFinger = true;
        }
      }
      else
      {
        //send the finger velocity command
        cartesian_cmd.publish(fingerCmd);
        stopMessageSentFinger = false;
      }
      break;
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "jaco_joy_teleop");

  // initialize the joystick controller
  jaco_joy_teleop controller;

  ros::Rate loop_rate(60);	//rate at which to publish velocity commands
  while (ros::ok())
  {
    controller.publish_velocity();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
