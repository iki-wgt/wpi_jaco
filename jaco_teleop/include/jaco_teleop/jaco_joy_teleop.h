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

#ifndef JACO_JOY_TELEOP_H_
#define JACO_JOY_TELEOP_H_

#include <ros/ros.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <sensor_msgs/Joy.h>

//Control modes
#define ARM_CONTROL 0 
#define FINGER_CONTROL 1

//Joystick types
#define ANALOG 0 //analog triggers
#define DIGITAL 1 //digital triggers

/*!
 * \def MAX_TRANS_VEL
 *
 * The maximum translational velocity.
 */
#define MAX_TRANS_VEL .175

/*!
 * \def MAX_FINGER_VEL
 * The maximum velocity for a finger.
 */
#define MAX_FINGER_VEL 30

/*!
 * \def MAX_ANG_VEL
 *
 * The maximum angular velocity.
 */
#define MAX_ANG_VEL 1.047

/*!
 * \class jaco_joy_teleop
 * \brief Allows for control of the JACO arm with a joystick.
 *
 * jaco_joy_teleop creates a ROS node that allows for the control of the
 * JACO arm with a joystick. This node listens to a /joy topic.
 */
class jaco_joy_teleop
{
public:
  /*!
   * \brief Constructor
   *
   * Creates a jaco_joy_teleop object that can be used control the JACO arm
   * with a joystick. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  jaco_joy_teleop();

  /*!
   * \brief Periodically publish velocity message to the arm controller
   */
  void publish_velocity();

private:
  /*!
   * \brief Joy topic callback function.
   *
   * \param joy the message for the joy topic
   */
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher cartesian_cmd; /*!< cartesian arm command topic */
  ros::Subscriber joy_sub; /*!< the joy topic */

  wpi_jaco_msgs::CartesianCommand fingerCmd; /*!< angular movement command */
  wpi_jaco_msgs::CartesianCommand cartesianCmd; /*!< cartesian movement command */

  int mode; /*!< the control mode */
  int controllerType; /*!< the type of joystick controller */
  double linear_throttle_factor; /*!< factor for reducing the linear speed */
  double angular_throttle_factor; /*!< factor for reducing the angular speed */
  double finger_throttle_factor; /*!< factor for reducing the finger speed */
  bool stopMessageSentArm; /*!< flag to prevent the arm stop command from being sent repeatedly when the controller is in the neutral position */
  bool stopMessageSentFinger; /*!< flag to prevent the finger stop command from being sent repeatedly when the controller is in the neutral position */
  bool initLeftTrigger; /*!< flag for whether the left trigger is initialized */
  bool initRightTrigger; /*!< flag for whether the right trigger is initialized */
  bool calibrated; /*!< flag for whether the controller is calibrated, this only affects controllers with analog triggers */
  bool EStopEnabled; /*!< software emergency stop for the arm*/
  bool helpDisplayed; /*!< flag so help is not repeatedly displayed*/
};

/*!
 * \brief Creates and runs the jaco_joy_teleop node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
