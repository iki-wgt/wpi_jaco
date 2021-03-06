/*!
 * \jaco_arm_trajectory_node.h
 * \brief Provides for trajectory execution and gripper control of the JACO arm.
 *
 * jaco_arm_trajectory_node creates a ROS node that provides trajectory execution and gripper 
 * control through the Kinova API, and smooth trajectory following through a velocity controller.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Mitchell Wills, WPI - mwills@wpi.edu
 */

#ifndef JACO_ARM_TRAJECTORY_NODE_H_
#define JACO_ARM_TRAJECTORY_NODE_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/foreach.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <ecl/geometry.hpp>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/HomeArmAction.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <jaco_sdk/Kinova.API.UsbCommandLayerUbuntu.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

/*
#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1.0472 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 1.5 //keep the trajectory at a followable speed
*/

#define LARGE_ACTUATOR_VELOCITY 0.5 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 3 //keep the trajectory at a followable speed

#define GRIPPER_CLOSED 0.7
#define GRIPPER_OPEN 0

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

//gains for trajectory follower
#define KP 300.0
#define KV 20.0
#define ERROR_THRESHOLD .03 //threshold in radians for combined joint error to consider motion a success

//control types
#define ANGULAR_CONTROL 1
#define CARTESIAN_CONTROL 2

namespace jaco
{

/*!
 * \class jacoArmTrajectoryController
 * \brief Provides for trajectory execution and gripper control of the JACO arm.
 *
 * jacoArmTrajectoryController creates a ROS node that provides trajectory execution and gripper 
 * control through the Kinova API, and smooth trajectory following through a velocity controller.
 */
class JacoArmTrajectoryController
{
private:
  // Messages
  ros::Publisher joint_state_pub_; //!< publisher for joint states
  ros::Publisher cartesianCmdPublisher; //!< publisher for Cartesian arm commands
  ros::Publisher angularCmdPublisher; //!< publisher for angular arm commands
  ros::Subscriber cartesianCmdSubscriber; //!< subscriber for Cartesian arm commands
  ros::Subscriber angularCmdSubscriber; //!< subscriber for angular arm commands

  // Services
  ros::ServiceClient jaco_fk_client; //!< forward kinematics client
  ros::ServiceClient qe_client; //!< quaternion to euler (XYZ) conversion client
  ros::ServiceServer cartesianPositionServer; //!< service server to get end effector pose
  ros::ServiceServer startApiControlServer; //!< start api control
  ros::ServiceServer stopApiControlServer; //!< stop api control


  ros::Timer joint_state_timer_; //!< timer for joint state publisher

  // Actionlib
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_server_; //!< point-to-point trajectory follower
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_trajectory_server_; //!< smooth point-to-point trajectory follower based on Cartesian end effector positions
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_joint_trajectory_server; //!< smooth point-to-point trajectory follower based on joint velocity control
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_server_; //!< gripper command action server
  actionlib::SimpleActionServer<wpi_jaco_msgs::HomeArmAction> home_arm_server;

  boost::recursive_mutex api_mutex;
  
  double max_curvature;

public:
  /**
   * \brief Constructor
   * @param nh ROS node handle
   * @param pnh ROS private node handle
   */
  JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * \brief Destructor
   */
  virtual ~JacoArmTrajectoryController();

  /**
   * \brief Reads joint states from the arm and publishes them as a JointState message
   */
  void update_joint_states();

  /**
   * \brief move the arm to the home position
   * @param goal action goal
   */
  void home_arm(const wpi_jaco_msgs::HomeArmGoalConstPtr &goal);

  /**
   * \brief Callback for the arm_controller, executes a joint angle trajectory
   * @param goal action goal
   */
  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  /**
   * \brief Callback for the smooth_arm_controller, executes a smoother Cartesian trajectory 
   
   * The trajectory is generated by converting joint angle trajectories to 
   * end effector cartesian trajectories and smoothed automatically by the JACO's
   * Cartesian position controller.
   
   * NOTE: the trajectories must not fall within constraints defined internally
   * on the JACO for singularity avoidance
   * @param goal action goal
   */
  void execute_smooth_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  /**
   * \brief Callback for the joint_velocity_controller, executes a smoothed trajectory with velocity control
   *
   * The trajectory is generated by interpolating a set of joint trajectory points and smoothing the corners
   * using an acceleration constraint. The trajectory is then followed by using a velocity controller implemented
   * in this node which sends joint velocity commands to the arm.
   * @param goal action goal
   */
  void execute_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  /**
   * \brief Callback for the gripper_server_, executes a gripper command
   * @param goal action goal
   */
  void execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal);

private:
  std::vector<std::string> joint_names;
  double joint_pos[NUM_JOINTS];
  double joint_vel[NUM_JOINTS];
  double joint_eff[NUM_JOINTS];

  unsigned int controlType; //current state of control

  /**
   * \brief Callback for sending an angular command to the arm
   * @param msg angular command and info
   */
  void angularCmdCallback(const wpi_jaco_msgs::AngularCommand& msg);

  /**
   * \brief Callback for sending a Cartesian command to the arm
   * @param msg Cartesian command and info
   */
  void cartesianCmdCallback(const wpi_jaco_msgs::CartesianCommand& msg);

  /**
   *\brief Stripped-down angular trajectory point sending to the arm
   *
   * This is designed for trajectory followers, which need a quick response
   * @param point angular trajectory point to send to the arm
   * @param erase if true, clear the trajectory point stack before sending point
   */
  void executeAngularTrajectoryPoint(TrajectoryPoint point, bool erase);

  /**
   * \brief Stripped-down Cartesian trajectory point sending to the arm
   *
   * This is designed for trajectory followers, which need a quick response
   * trajectory followers that need very quick response
   * @param point Cartesian trajectory point to send to the arm
   * @param erase if true, clear the trajectory point stack before sending point
   */
  void executeCartesianTrajectoryPoint(TrajectoryPoint point, bool erase);

  /**
   * \brief Service callback for getting the current Cartesian pose of the end effector
   *
   * This allows other nodes to get the pose which is normally only accessible
   * through the Kinova API
   * @param req empty service request
   * @param res service response including the end effector pose
   * @return true on success
   */
  bool getCartesianPosition(wpi_jaco_msgs::GetCartesianPosition::Request &req,
                            wpi_jaco_msgs::GetCartesianPosition::Response &res);

   /**
   * \brief Start the API control to control the arm after API control was lost
   * because the joystick was used or stopAPIControlw as called
   *
   * @return true on success
   */
  bool startApiControl(std_srvs::Empty::Request& request, 
                                                  std_srvs::Empty::Response& response);


   /**
   * \brief Stop the API control as e-stop for the arm. Do move the arm again startApiControl
   * has to be called.
   *
   * @return true on success
   */
  bool stopApiControl(std_srvs::Empty::Request& request, 
                                                  std_srvs::Empty::Response& response);

};

}

#endif
