#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>         //関節の位置？
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <human_navigation/HumanNaviMsg.h>
#include <human_navigation/HumanNaviGuidanceMsg.h>

class HSRKeyTeleop
{
private:
//  static const char KEYCODE_0 = 0x30;
  static const char KEYCODE_9 = 0x39;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEYCODE_A = 0x61;
  static const char KEYCODE_B = 0x62;
  static const char KEYCODE_C = 0x63;
  static const char KEYCODE_D = 0x64;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_I = 0x69;
  static const char KEYCODE_J = 0x6a;
  static const char KEYCODE_K = 0x6b;
  static const char KEYCODE_L = 0x6c;
  static const char KEYCODE_M = 0x6d;
  static const char KEYCODE_N = 0x6e;
  static const char KEYCODE_O = 0x6f;
  static const char KEYCODE_Q = 0x71;
  static const char KEYCODE_T = 0x74;
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;

  static const char KEYCODE_COMMA  = 0x2c;
  static const char KEYCODE_PERIOD = 0x2e;
  static const char KEYCODE_SPACE  = 0x20;

  const std::string ARM_LIFT_JOINT_NAME = "arm_lift_joint";

  const std::string MSG_ARE_YOU_READY  = "Are_you_ready?";

  const std::string MSG_I_AM_READY      = "I_am_ready";
  const std::string MSG_GIVE_UP         = "Give_up";

public:
  HSRKeyTeleop();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void jointStateCallbackRobot1(const sensor_msgs::JointState::ConstPtr& joint_state);
  void sendMessageRobot1(const std::string &message);
  void moveBaseTwistRobot1(double linear_x, double linear_y, double angular_z);
  void moveBaseJointTrajectoryRobot1(double linear_x, double linear_y, double theta, double duration_sec);
  void operateArmRobot1(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const double duration_sec);
  void operateArmRobot1(const std::string &name, const double position, const double duration_sec);
  void operateArmFlexRobot1(const double arm_flex_pos, const double wrist_flex_pos);
  double getDurationRot1(const double next_pos, const double current_pos);
  void operateHand(bool grasp);
  void sendGuidanceMessage(const std::string &message, const std::string displayType);

  void showHelp();
  int run(int argc, char **argv);

private://ロボット1、ロボット２の変数　追記日7/4
  // Last position and previous position of arm_lift_joint
  double arm_lift_joint_pos1_robot1_;
  double arm_lift_joint_pos2_robot1_;
  double arm_flex_joint_pos_robot1_;
  double wrist_flex_joint_pos_robot1_;

 /* double arm_lift_joint_pos1_robot2_;
  double arm_lift_joint_pos2_robot2_;
  double arm_flex_joint_pos_robot2_;
  double wrist_flex_joint_pos_robot2_;*/

  ros::NodeHandle node_handle_robot1_;
  ros::Publisher  pub_msg_robot1_;
  ros::Subscriber sub_joint_state_robot1_;
  ros::Publisher  pub_base_twist_robot1_;
  ros::Publisher  pub_base_trajectory_robot1_;
  ros::Publisher  pub_arm_trajectory_robot1_;
  ros::Publisher  pub_gripper_trajectory_robot1_;
  ros::Publisher  pub_guidance_msg_robot1_;

 /* ros::NodeHandle node_handle_robot2_;
  ros::Publisher  pub_msg_robot2_;
  ros::Subscriber sub_joint_state_robot2_;
  ros::Publisher  pub_base_twist_robot2_;
  ros::Publisher  pub_base_trajectory_robot2_;
  ros::Publisher  pub_arm_trajectory_robot2_;
  ros::Publisher  pub_gripper_trajectory_robot2_;
  ros::Publisher  pub_guidance_msg_robot2_;*/

  tf::TransformListener listener_;
};

//ロボ1初期化
HSRKeyTeleop::HSRKeyTeleoprobot1()
{
  arm_lift_joint_pos1_robot1_  = 0.0;
  arm_lift_joint_pos2_robot1_ = 0.0;
  arm_flex_joint_pos_robot1_ = 0.0;
  wrist_flex_joint_pos_robot1_ = 0.0;
}

//ロボ2初期化
//HSRKeyTeleop::HSRKeyTeleoprobot2()
//{
//    arm_lift_joint_pos1_robot2_ = 0.0;
//    arm_lift_joint_pos2_robot2_ = 0.0;
//    arm_flex_joint_pos_robot2_ = 0.0;
//    wrist_flex_joint_pos_robot2_ = 0.0;
//}


void HSRKeyTeleop::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int HSRKeyTeleop::canReceive( int fd )
{
  fd_set fdset;
  int ret;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}
//ロボット１のコールバック関数
void HSRKeyTeleop::jointStateCallbackRobot1(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  for(int i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i] == "arm_lift_joint_robot1")
    {
      arm_lift_joint_pos2_robot1_ = arm_lift_joint_pos1_robot1_;
      arm_lift_joint_pos1_robot1_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "arm_flex_joint_robot1")
    {
      arm_flex_joint_pos_robot1_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "wrist_flex_joint_robot1")
    {
      wrist_flex_joint_pos_robot1_ = jroint_state->position[i];
    }
  }
}

//ロボット2のコールバック関数    7/7
//void HSRKeyTeleop::jointStateCallbackRobot2(const sensor_msgs::JointState::ConstPtr& joint_state)
//{
//    for (int i = 0; i < joint_state->name.size(); i++)
//    {
//        if (joint_state->name[i] == "arm_lift_joint_robot2")
//        {
//            arm_lift_joint_pos2_robot2_= arm_lift_joint_pos1_robot2_;
//            arm_lift_joint_pos1_robot2_ = joint_state->position[i];
//        }
//        if (joint_state->name[i] == "arm_flex_jointrobot2")
//        {
//            arm_flex_joint_pos_robot2_ = joint_state->position[i];
//        }
//        if (joint_state->name[i] == "wrist_flex_jointrobot2")
//        {
//            wrist_flex_joint_pos_robot2_ = joint_state->position[i];
//        }
//    }
//}

void HSRKeyTeleop::sendMessageRobot1(const std::string &message)
{
  ROS_INFO("Send message:%s", message.c_str());

  human_navigation::HumanNaviMsg human_navi_msg;
  human_navi_msg.message = message;
  pub_msg_robot1_.publish(human_navi_msg);
}

//必要かどうかわからない
//void HSRKeyTeleop::sendMessagerobot2(const std::string& message)
//{
//    ROS_INFO("Send message:%s", message.c_str());
//
//    human_navigation::HumanNaviMsg human_navi_msg;
//    human_navi_msg.message = message;
//    pub_msg_robot2_.publish(human_navi_msg);
//}
///

void HSRKeyTeleop::moveBaseTwistRobot1(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_robot1_.publish(twist);
}

//必要？
//void HSRKeyTeleop::moveBaseTwistRobot2(double linear_x, double linear_y, double angular_z)
//{
//    geometry_msgs::Twist twist;
//
//    twist.linear.x = linear_x;
//    twist.linear.y = linear_y;
//    twist.angular.z = angular_z;
//    pub_base_twist_robot2_.publish(twist);
//}

//ロボ1のジョイント
void HSRKeyTeleop::moveBaseJointTrajectoryRobot1(double linear_x, double linear_y, double theta, double duration_sec)
{
  if(listener_.canTransform("/odom", "/base_footprint", ros::Time(0)) == false)
  {
    return;
  }

  geometry_msgs::PointStamped basefootprint_2_target;
  geometry_msgs::PointStamped odom_2_target;
  basefootprint_2_target.header.frame_id = "/base_footprint";
  basefootprint_2_target.header.stamp = ros::Time(0);
  basefootprint_2_target.point.x = linear_x;
  basefootprint_2_target.point.y = linear_y;
  listener_.transformPoint("/odom", basefootprint_2_target, odom_2_target);

  tf::StampedTransform transform;
  listener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
  tf::Quaternion currentRotation = transform.getRotation();
  tf::Matrix3x3 mat(currentRotation);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("odom_x");
  joint_trajectory.joint_names.push_back("odom_y");
  joint_trajectory.joint_names.push_back("odom_t");

  trajectory_msgs::JointTrajectoryPoint omni_joint_point;
  omni_joint_point.positions = {odom_2_target.point.x, odom_2_target.point.y, yaw + theta};
  omni_joint_point.time_from_start = ros::Duration(duration_sec);

  joint_trajectory.points.push_back(omni_joint_point);
  pub_base_trajectory_.publish(joint_trajectory);
}

//ロボ２のジョイント　現在まだ中身変更していない 変更する必要あり
//void HSRKeyTeleop::moveBaseJointTrajectoryRobot2(double linear_x, double linear_y, double theta, double duration_sec)
//{
//    if (listener_.canTransform("/odom", "/base_footprint", ros::Time(0)) == false)
//    {
//        return;
//    }
//
//    geometry_msgs::PointStamped basefootprint_2_target;
//    geometry_msgs::PointStamped odom_2_target;
//    basefootprint_2_target.header.frame_id = "/base_footprint";
//    basefootprint_2_target.header.stamp = ros::Time(0);
//    basefootprint_2_target.point.x = linear_x;
//    basefootprint_2_target.point.y = linear_y;
//    listener_.transformPoint("/odom", basefootprint_2_target, odom_2_target);
//
//    tf::StampedTransform transform;
//    listener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
//    tf::Quaternion currentRotation = transform.getRotation();
//    tf::Matrix3x3 mat(currentRotation);
//    double roll, pitch, yaw;
//    mat.getRPY(roll, pitch, yaw);
//
//    trajectory_msgs::JointTrajectory joint_trajectory;
//    joint_trajectory.joint_names.push_back("odom_x");
//    joint_trajectory.joint_names.push_back("odom_y");
//    joint_trajectory.joint_names.push_back("odom_t");
//
//    trajectory_msgs::JointTrajectoryPoint omni_joint_point;
//    omni_joint_point.positions = { odom_2_target.point.x, odom_2_target.point.y, yaw + theta };
//    omni_joint_point.time_from_start = ros::Duration(duration_sec);
//
//    joint_trajectory.points.push_back(omni_joint_point);
//    pub_base_trajectory_.publish(joint_trajectory);
//}


void HSRKeyTeleop::operateArmRobot1(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const double duration_sec)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("arm_lift_joint_robot1");
  joint_trajectory.joint_names.push_back("arm_flex_joint_robot1");
  joint_trajectory.joint_names.push_back("arm_roll_joint_robot1");
  joint_trajectory.joint_names.push_back("wrist_flex_joint_robot1");
  joint_trajectory.joint_names.push_back("wrist_roll_joint_robot1");

  trajectory_msgs::JointTrajectoryPoint arm_joint_point_robot1;

  arm_joint_point_robot1.positions = {arm_lift_pos, arm_flex_pos, 0.0f, wrist_flex_pos, 0.0f};

  arm_joint_point_robot1.time_from_start = ros::Duration(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point_robot1);
  pub_arm_trajectory_.publish(joint_trajectory);
}

//void HSRKeyTeleop::operateArmRobot2(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const double duration_sec)
//{
//    trajectory_msgs::JointTrajectory joint_trajectory;
//    joint_trajectory.joint_names.push_back("arm_lift_joint_robot2");
//    joint_trajectory.joint_names.push_back("arm_flex_joint_robot2");
//    joint_trajectory.joint_names.push_back("arm_roll_joint_robot2");
//    joint_trajectory.joint_names.push_back("wrist_flex_joint_robot2");
//    joint_trajectory.joint_names.push_back("wrist_roll_joint_robot2");
//
//    trajectory_msgs::JointTrajectoryPoint arm_joint_point_robot2;
//
//    arm_joint_point_robot2.positions = { arm_lift_pos, arm_flex_pos, 0.0f, wrist_flex_pos, 0.0f };
//
//    arm_joint_point_robot2.time_from_start = ros::Duration(duration_sec);
//    joint_trajectory.points.push_back(arm_joint_point_robot2);
//    pub_arm_trajectory_.publish(y);
//}

//ロボ1のアーム関係
void HSRKeyTeleop::operateArmRobot1(const std::string &name, const double position, const double duration_sec)
{
  if(name == "arm_lift_joint_robot1")
  {
    this->operateArmRobot1(position, arm_flex_joint_pos_robot1_, wrist_flex_joint_pos_robot1_, duration_sec);
  }
  else if(name == "arm_flex_joint_robot1")
  {
    this->operateArmRobot1(2.0*arm_lift_joint_pos1_robot1_ -arm_lift_joint_pos2_robot1_, position, wrist_flex_joint_pos_robot1_, duration_sec);
  }
  else if(name == "wrist_flex_joint_robot1")
  {
    this->operateArmRobot1(2.0*arm_lift_joint_pos1_robot1_-arm_lift_joint_pos2_robot1_, arm_flex_joint_pos_robot1_, position, duration_sec);
  }
}

void HSRKeyTeleop::operateArmFlexRobot1(const double arm_flex_pos, const double wrist_flex_pos)
{
  double duration1 = std::max1(this->getDurationRot1(arm_flex_pos, arm_flex_joint_pos_robot1_), this->getDurationRot1(wrist_flex_pos, wrist_flex_joint_pos_robot1_));

  this->operateArmRobot1(2.0*arm_lift_joint_pos1_robot1_ -arm_lift_joint_pos2_robot1_, arm_flex_pos, wrist_flex_pos, duration1);
}

//ロボット２のアーム関係
//void HSRKeyTeleop::operateArmRobot2(const std::string& name, const double position, const double duration_sec)
//{
//    if (name == "arm_lift_joint_robot2")
//    {
//        this->operateArmRobot2(position, arm_flex_joint_pos_robot2_, wrist_flex_joint_pos_robot2_, duration_sec);
//    }
//    else if (name == "arm_flex_joint_robot2")
//    {
//        this->operateArmRobot2(2.0 * arm_lift_joint_pos1_robot2_ - arm_lift_joint_pos2_robot2_, position, wrist_flex_joint_pos_robot2_, duration_sec);
//    }
//    else if (name == "wrist_flex_joint_robot2")
//    {
//        this->operateArmRobot2(2.0 * arm_lift_joint_pos1_robot2_ - arm_lift_joint_pos2_robot2_, arm_flex_joint_pos_robot2_, position, duration_sec);
//    }
//}
//
//void HSRKeyTeleop::operateArmFlexRobot2(const double arm_flex_pos, const double wrist_flex_pos)
//{
//    double duration2 = std::max2(this->getDurationRot2(arm_flex_pos, arm_flex_joint_pos_robot2_), this->getDurationRot2(wrist_flex_pos, wrist_flex_joint_pos_robot2_));
//
//    this->operateArmRobot2(2.0 * arm_lift_joint_pos1_robot2_ - arm_lift_joint_pos2_robot2_, arm_flex_pos, wrist_flex_pos, duration2);
//}


//ロボ1
double HSRKeyTeleop::getDurationRot1(const double next_pos, const double current_pos)
{
  return std::max1<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
}

//ロボ2
//double HSRKeyTeleop::getDurationRot2(const double next_pos, const double current_pos)
//{
//    return std::max2<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
//}


//ロボ1手動かす
void HSRKeyTeleop::operateHandRobot1(bool is_hand_open)
{
  std::vector<std::string> joint_names {"hand_motor_joint_robot1"};
  std::vector<double> positions;

  if(is_hand_open)
  {
    ROS_DEBUG("Grasp");
    positions.push_back(-0.105);
  }
  else
  {
    ROS_DEBUG("Open hand");
    positions.push_back(+1.239);
  }

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = ros::Duration(2);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);
  pub_gripper_trajectory_.publish(joint_trajectory);
}

//ロボ2手動かす
//void HSRKeyTeleop::operateHandRobot2(bool is_hand_open)
//{
//    std::vector<std::string> joint_names{ "hand_motor_joint_robot2" };
//    std::vector<double> positions;
//
//    if (is_hand_open)
//    {
//        ROS_DEBUG("Grasp");
//        positions.push_back(-0.105);
//    }
//    else
//    {
//        ROS_DEBUG("Open hand");
//        positions.push_back(+1.239);
//    }
//
//    trajectory_msgs::JointTrajectoryPoint point;
//    point.positions = positions;
//    point.time_from_start = ros::Duration(2);
//
//    trajectory_msgs::JointTrajectory joint_trajectory;
//    joint_trajectory.joint_names = joint_names;
//    joint_trajectory.points.push_back(point);
//    pub_gripper_trajectory_.publish(joint_trajectory);
//}


//1つのみ
void HSRKeyTeleop::sendGuidanceMessage(const std::string &message, const std::string displayType)
{
  human_navigation::HumanNaviGuidanceMsg guidanceMessage;
  guidanceMessage.message = message;
  guidanceMessage.display_type = displayType;
  pub_guidance_msg_.publish(guidanceMessage);

  ROS_INFO("Send guide message: %s : %s", guidanceMessage.message.c_str(), guidanceMessage.display_type.c_str());
}

void HSRKeyTeleop::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR");
  puts("space      : Stop HSR");
  puts("---------------------------");
  puts("Move HSR Linearly (1m)");
  puts("  u   i   o  ");
  puts("  j   k   l  ");
  puts("  m   ,   .  ");
  puts("---------------------------");
  puts("q/z : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("y : Up   Torso");
  puts("h : Stop Torso");
  puts("n : Down Torso");
  puts("---------------------------");
  puts("a : Rotate Arm - Vertical");
  puts("b : Rotate Arm - Upward");
  puts("c : Rotate Arm - Horizontal");
  puts("d : Rotate Arm - Downward");
  puts("---------------------------");
  puts("g : Grasp/Open Hand");
  puts("---------------------------");
  puts("t : Send Test Message");
  //puts(("0 : Send "+MSG_I_AM_READY).c_str());
  puts(("9 : Send "+MSG_GIVE_UP).c_str());
}

int HSRKeyTeleop::run(int argc, char **argv)
{
  char c;

  /////////////////////////////////////////////
  // get the console in raw mode
  int kfd = 0;
  struct termios cooked;

  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  showHelp();

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(40);
//ロボット１の情報
  std::string pub_msg_to_moderator_topic_name_robot1;
  std::string sub_joint_state_topic_name_robot1;   //launchファイルに書かれている
  std::string pub_base_twist_topic_name_robot1;    //launchファイルに書かれている
  std::string pub_base_trajectory_topic_name_robot1;       //launchファイルに書かれている
  std::string pub_arm_trajectory_topic_name_robot1;
  std::string pub_gripper_trajectory_topic_name_robot1;    //launchファイルに書かれている
  //ローカル変数に格納
  node_handle_.param<std::string>("pub_msg_to_moderator_topic_name_robot1",   pub_msg_to_moderator_topic_name_robot1,   "/human_navigation/message/to_moderator");  //Unity側では1つしかない

  node_handle_.param<std::string>("sub_joint_state_topic_name_robot1",        sub_joint_state_topic_name_robot1,        "/hsr1/hsrb/joint_states");
  node_handle_.param<std::string>("pub_base_twist_topic_name_robot1",         pub_base_twist_topic_name_robot1,         "/hsr1/hsrb/command_velocity");
  node_handle_.param<std::string>("pub_base_trajectory_topic_name_robot1",    pub_base_trajectory_topic_name_robot1,    "/hsr1/hsrb/omni_base_controller/command");
  node_handle_.param<std::string>("pub_arm_trajectory_topic_name_robot1",     pub_arm_trajectory_topic_name_robot1,     "/hsr1/hsrb/arm_trajectory_controller/command");
  node_handle_.param<std::string>("pub_gripper_trajectory_topic_name_robot1", pub_gripper_trajectory_topic_name_robot1, "/hsr1/hsrb/gripper_controller/command");

  pub_msg_                = node_handle_.advertise<human_navigation::HumanNaviMsg>(pub_msg_to_moderator_topic_name_robot1, 10);

  sub_joint_state_robot1        = node_handle_.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name_robot1, 10, &HSRKeyTeleop::jointStateCallbackRobot1, this);
  pub_base_twist_robot1         = node_handle_.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name_robot1, 10);
  pub_base_trajectory_robot1    = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_base_trajectory_topic_name_robot1, 10);
  pub_arm_trajectory_robot1     = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name_robot1, 10);
  pub_gripper_trajectory_robot1 = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name_robot1, 10);
  pub_guidance_msg_robot1       = node_handle_.advertise<human_navigation::HumanNaviGuidanceMsg>("/hsr1/human_navigation/message/guidance_message", 10); 

  //ロボット2の情報
  //std::string pub_msg_to_moderator_topic_name_robot2;
  //std::string sub_joint_state_topic_name_robot2;   //launchファイルに書かれている
  //std::string pub_base_twist_topic_name_robot2;    //launchファイルに書かれている
  //std::string pub_base_trajectory_topic_name_robot2;       //launchファイルに書かれている
  //std::string pub_arm_trajectory_topic_name_robot2;
  //std::string pub_gripper_trajectory_topic_name_robot2;    //launchファイルに書かれている
  ////ローカル変数に格納
  //node_handle_.param<std::string>("pub_msg_to_moderator_topic_name_robot2", pub_msg_to_moderator_topic_name_robot1, "/human_navigation/message/to_moderator");  //Unity側では1つしかない

  //node_handle_.param<std::string>("sub_joint_state_topic_name_robot2", sub_joint_state_topic_name_robot2, "/hsr2/hsrb/joint_states");
  //node_handle_.param<std::string>("pub_base_twist_topic_name_robot2", pub_base_twist_topic_name_robot2, "/hsr2/hsrb/command_velocity");
  //node_handle_.param<std::string>("pub_base_trajectory_topic_name_robot2", pub_base_trajectory_topic_name_robot2, "/hsr2/hsrb/omni_base_controller/command");
  //node_handle_.param<std::string>("pub_arm_trajectory_topic_name_robot2", pub_arm_trajectory_topic_name_robot2, "/hsr2/hsrb/arm_trajectory_controller/command");
  //node_handle_.param<std::string>("pub_gripper_trajectory_topic_name_robot2", pub_gripper_trajectory_topic_name_robot2, "/hsr2/hsrb/gripper_controller/command");

  //pub_msg_ = node_handle_.advertise<human_navigation::HumanNaviMsg>(pub_msg_to_moderator_topic_name_robot2, 10);

  //sub_joint_state_robot2 = node_handle_.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name_robot2, 10, &HSRKeyTeleop::jointStateCallbackRobot2, this);
  //pub_base_twist_robot2 = node_handle_.advertise<geometry_msgs::Twist>(pub_base_twist_topic_name_robot2, 10);
  //pub_base_trajectory_robot2 = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_base_trajectory_topic_name_robot2, 10);
  //pub_arm_trajectory_robot2 = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name_robot2, 10);
  //pub_gripper_trajectory_robot2 = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name_robot2, 10);
  //pub_guidance_msg_robot2 = node_handle_.advertise<human_navigation::HumanNaviGuidanceMsg>("/hsr2/human_navigation/message/guidance_message", 10);
//ロボット１の情報


  const float linear_coef_robot1         = 0.2f;
  const float linear_oblique_coef_robot1 = 0.141f;
  const float angular_coef_robot1        = 0.5f;

  float move_speed_robot1 = 1.0f;
  bool is_hand_open_robot1 = false;

  std::string arm_lift_joint_name_robot1   = "arm_lift_joint_robot1_";
  std::string arm_flex_joint_name_robot1   = "arm_flex_joint_robot1_";
  std::string wrist_flex_joint_name_robot1 = "wrist_flex_joint_robot1_";

  //ロボ2の情報
  //const float linear_coef_robot2 = 0.2f;
  //const float linear_oblique_coef_robot2 = 0.141f;
  //const float angular_coef_robot2 = 0.5f;

  //float move_speed_robot2 = 1.0f;
  //bool is_hand_open_robot2 = false;

  //std::string arm_lift_joint_name_robot2 = "arm_lift_joint_robot2_";
  //std::string arm_flex_joint_name_robot2 = "arm_flex_joint_robot2_";
  //std::string wrist_flex_joint_name_robot2 = "wrist_flex_joint_robot2_";


  while (ros::ok())
  {
    if(canReceive(kfd))
    {
      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      switch(c)
      {
        //case KEYCODE_0:
        //{
        //  sendMessage(pub_msg, MSG_I_AM_READY);
        //  break;
        //}
        case KEYCODE_9:
        {
          sendMessage(MSG_GIVE_UP);
          break;
        }
        case KEYCODE_UP:
        {
          ROS_DEBUG("Go Forward");
          moveBaseTwistRobot1(+linear_coef_robot1 *move_speed_robot1, 0.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go Backward");
          moveBaseTwistRobot1(-linear_coef_robot1 *move_speed_robot1, 0.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Go Right");
          moveBaseTwistRobot1(0.0, 0.0, -angular_coef_robot1 *move_speed_robot1);
          break;
        }
        case KEYCODE_LEFT:
        {
          ROS_DEBUG("Go Left");
          moveBaseTwistRobot1(0.0, 0.0, +angular_coef_robot1 *move_speed_robot1);
          break;
        }
        case KEYCODE_SPACE:
        {
          ROS_DEBUG("Stop");
          moveBaseTwistRobot1(0.0, 0.0, 0.0);
          break;
        }
        case KEYCODE_U:
        {
          ROS_DEBUG("Move Left Forward");
          moveBaseJointTrajectoryRobot1(+1.0, +1.0, +M_PI_4, 10);
          break;
        }
        case KEYCODE_I:
        {
          ROS_DEBUG("Move Forward");
          moveBaseJointTrajectoryRobot1(+1.0, 0.0, 0.0, 10);
          break;
        }
        case KEYCODE_O:
        {
          ROS_DEBUG("Move Right Forward");
          moveBaseJointTrajectoryRobot1(+1.0, -1.0, -M_PI_4, 10);
          break;
        }
        case KEYCODE_J:
        {
          ROS_DEBUG("Move Left");
          moveBaseJointTrajectoryRobot1(0.0, +1.0, +M_PI_2, 10);
          break;
        }
        case KEYCODE_K:
        {
          ROS_DEBUG("Stop");
          moveBaseJointTrajectoryRobot1(0.0, 0.0, 0.0, 0.5);
          break;
        }
        case KEYCODE_L:
        {
          ROS_DEBUG("Move Right");
          moveBaseJointTrajectoryRobot1(0.0, -1.0, -M_PI_2, 10);
          break;
        }
        case KEYCODE_M:
        {
          ROS_DEBUG("Move Left Backward");
          moveBaseJointTrajectoryRobot1(-1.0, +1.0, +M_PI_2+M_PI_4, 10);
          break;
        }
        case KEYCODE_COMMA:
        {
          ROS_DEBUG("Move Backward");
          moveBaseJointTrajectoryRobot1(-1.0, 0.0, +M_PI, 10);
          break;
        }
        case KEYCODE_PERIOD:
        {
          ROS_DEBUG("Move Right Backward");
          moveBaseJointTrajectoryRobot1(-1.0, -1.0, -M_PI_2-M_PI_4, 10);
          break;
        }
        case KEYCODE_Q:
        {
          ROS_DEBUG("Move Speed Up");
          move_speed *= 2;      //スピードを2倍にする
          if(move_speed > 2  ){ move_speed=2; }     //2倍以上にはしない
          break;
        }
        case KEYCODE_Z:
        {
          ROS_DEBUG("Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case KEYCODE_Y:
        {
          ROS_DEBUG("Up Torso");
          operateArm(arm_lift_joint_name, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_robot1_) / 0.05), 1));
          break;
        }
        case KEYCODE_H:
        {
          ROS_DEBUG("Stop Torso");
          operateArm(arm_lift_joint_name, 2.0*arm_lift_joint_pos1_robot1_ -arm_lift_joint_pos2_robot1_, 0.5);
          break;
        }
        case KEYCODE_N:
        {
          ROS_DEBUG("Down Torso");
          operateArm(arm_lift_joint_name, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_robot1_) / 0.05), 1));
          break;
        }
        //operateArm(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const int duration_sec);
        case KEYCODE_A:
        {
          ROS_DEBUG("Rotate Arm - Vertical");
          operateArmFlex(0.0, -1.57);
          break;
        }
        case KEYCODE_B:
        {
          ROS_DEBUG("Rotate Arm - Upward");
          operateArmFlex(-0.785, -0.785);
          break;
        }
        case KEYCODE_C:
        {
          ROS_DEBUG("Rotate Arm - Horizontal");
          operateArmFlex(-1.57, 0.0);
          break;
        }
        case KEYCODE_D:
        {
          ROS_DEBUG("Rotate Arm - Downward");
          operateArmFlex(-2.2, 0.35);
          break;
        }
        case KEYCODE_G:
        {
          operateHand(is_hand_open);

          is_hand_open = !is_hand_open;
          break;
        }
        case KEYCODE_T:
        {
          sendGuidanceMessage("This is a test message.", "All");
          break;
        }
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hsr_teleop_key");

  HSRKeyTeleop hsr_key_teleop;
  return hsr_key_teleop.run(argc, argv);
}
