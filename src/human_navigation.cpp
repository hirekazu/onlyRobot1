#include <stdio.h>
#include <termios.h>
#include <ros/ros.h>
#include <human_navigation/HumanNaviObjectInfo.h>
#include <human_navigation/HumanNaviDestination.h>
#include <human_navigation/HumanNaviTaskInfo.h>
#include <human_navigation/HumanNaviMsg.h>
#include <human_navigation/HumanNaviGuidanceMsg.h>
#include <human_navigation/HumanNaviAvatarStatus.h>
#include <human_navigation/HumanNaviObjectStatus.h>



#include <math.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class HumanNavigationSample
{


    std::vector<std::string> orderList =
    {
        "first",
        "second",
        "third",
        "force",
        "fifth",
        "sixth",
        "seventh",
        "eight",
        "ninth",
        "tenth"
    };

	enum Step
	{
		Initialize,
		Ready,
		WaitTaskInfo,
		Introduction,
		MoveToTargetInstruction,
    MoveToTarget,
    ReachToTarget,
		GuideForTakingObject,
		WaitGraspTarget,
		LookMeAgain,
		MoveToDestinationInstruction,
    MoveToDestination,
    ReachToDestination,
		GuideForDestination,
		WaitTaskFinished,
		TaskFinished
	};

	enum class SpeechState
	{
		None,
		WaitingState,
		Speaking,
		Speakable
	};

	// human navigation message from/to the moderator
	const std::string MSG_ARE_YOU_READY      = "Are_you_ready?";
	const std::string MSG_TASK_SUCCEEDED     = "Task_succeeded";
	const std::string MSG_TASK_FAILED        = "Task_failed";
	const std::string MSG_TASK_FINISHED      = "Task_finished";
	const std::string MSG_GO_TO_NEXT_SESSION = "Go_to_next_session";
	const std::string MSG_MISSION_COMPLETE   = "Mission_complete";
	const std::string MSG_REQUEST            = "Guidance_request";
	const std::string MSG_SPEECH_STATE       = "Speech_state";
	const std::string MSG_SPEECH_RESULT      = "Speech_result";

	const std::string MSG_I_AM_READY        = "I_am_ready";
	const std::string MSG_GET_AVATAR_STATUS = "Get_avatar_status";
	const std::string MSG_GET_OBJECT_STATUS = "Get_object_status";
	const std::string MSG_GET_SPEECH_STATE  = "Get_speech_state";

	// display type of guidance message panels for the avatar (test subject)
	const std::string DISPLAY_TYPE_ALL         = "All";
	const std::string DISPLAY_TYPE_ROBOT_ONLY  = "RobotOnly";
	const std::string DISPLAY_TYPE_AVATAR_ONLY = "AvatarOnly";
	const std::string DISPLAY_TYPE_NONE        = "None";

	int step;
	SpeechState speechState;

	bool isStarted;
	bool isFinished;

	bool isTaskInfoReceived;
	bool isRequestReceived;

	ros::Time timePrevSpeechStateConfirmed;

	bool isSentGetAvatarStatus;
	bool isSentGetObjectStatus;
	human_navigation::HumanNaviTaskInfo taskInfo;
	std::string guideMsg;

	human_navigation::HumanNaviAvatarStatus avatarStatus;
	human_navigation::HumanNaviObjectStatus objectStatus;
 //   HSRKeyTeleop hsr;

    double rad_target;
    double rad_destination;

    double kyori_from_avater;

    double arm_lift_joint_pos2_;
    double arm_lift_joint_pos1_;
    double arm_flex_joint_pos_;
    double wrist_flex_joint_pos_;
    int wrong_counter;
    std::vector<std::string> names;
    std::vector<double> positions;
    trajectory_msgs::JointTrajectory joint_trajectory;

    trajectory_msgs::JointTrajectoryPoint arm_joint_point;
    std::string orderMsg;
    std::string wayMsg;
    std::string destinationName;
    std::string targetName;
    std::string graspingName;
    std::string furnitureName;
    std::string onWayMsg;

    const double THETA_KYORI = 2.0;
    const double THETA_Z = 1.0;


	void init()
	{
		step = Initialize;
		speechState = SpeechState::None;
		reset();
	}

	void reset()
	{
		isStarted             = false;
		isFinished            = false;
		isTaskInfoReceived    = false;
		isRequestReceived     = false;
		isSentGetAvatarStatus = false;
		isSentGetObjectStatus = false;

    rad_target = 0;
    rad_destination = 0;
    kyori_from_avater = 0;
    arm_lift_joint_pos1_=0.0;
    arm_lift_joint_pos2_=0.0;
    arm_flex_joint_pos_   = 0.0;
    wrist_flex_joint_pos_ = 0.0;

    orderMsg = "";
    wayMsg = "";
    furnitureName = "";
    destinationName= "";
    targetName = "";
    graspingName = "";
    onWayMsg = "";
    wrong_counter = 0;
	}

	// send humanNaviMsg to the moderator (Unity)
	void sendMessage(ros::Publisher &publisher, const std::string &message)
	{
		human_navigation::HumanNaviMsg human_navi_msg;
		human_navi_msg.message = message;
		publisher.publish(human_navi_msg);

		ROS_INFO("Send message:%s", message.c_str());
	}

  void sendGuidanceMessage(ros::Publisher &publisher, const std::string &message, const std::string displayType)
	{
		human_navigation::HumanNaviGuidanceMsg guidanceMessage;
		guidanceMessage.message = message;
		guidanceMessage.display_type = displayType;
		guidanceMessage.source_language = ""; // Blank or ISO-639-1 language code, e.g. "en".
		guidanceMessage.target_language = ""; // Blank or ISO-639-1 language code, e.g. "ja".
		publisher.publish(guidanceMessage);

		speechState = SpeechState::Speaking;

		ROS_INFO("Send guide message: %s : %s", guidanceMessage.message.c_str(), guidanceMessage.display_type.c_str());
	}


	// receive humanNaviMsg from the moderator (Unity)
	void messageCallback(const human_navigation::HumanNaviMsg::ConstPtr& message)
	{
		ROS_INFO("Subscribe message: %s : %s", message->message.c_str(), message->detail.c_str());

		if(message->message==MSG_ARE_YOU_READY)
		{
			isStarted = true;
		}
		else if(message->message==MSG_REQUEST)
		{
			if(isTaskInfoReceived && !isFinished)
			{
				isRequestReceived = true;
			}
		}
		else if(message->message==MSG_TASK_SUCCEEDED)
		{
		}
		else if(message->message==MSG_TASK_FAILED)
		{
		}
		else if(message->message==MSG_TASK_FINISHED)
		{
			isFinished = true;
		}
		else if(message->message==MSG_GO_TO_NEXT_SESSION)
		{
			ROS_INFO("Go to next session");
			step = Initialize;
		}
		else if(message->message==MSG_MISSION_COMPLETE)
		{
			//exit(EXIT_SUCCESS);
		}
		else if(message->message==MSG_SPEECH_STATE)
		{
			if(message->detail=="Is_speaking")
			{
				speechState = SpeechState::Speaking;
			}
			else
			{
				speechState = SpeechState::Speakable;
			}
		}
		else if(message->message==MSG_SPEECH_RESULT)
		{
			ROS_INFO("Speech result: %s", message->detail.c_str());
		}
	}

	// receive taskInfo from the moderator (Unity)
	void taskInfoMessageCallback(const human_navigation::HumanNaviTaskInfo::ConstPtr& message)
	{
		taskInfo = *message;
        /*
		ROS_INFO_STREAM(
			"Subscribe task info message:" << std::endl <<
			"Environment ID: " << taskInfo.environment_id << std::endl <<
			"Target object: " << std::endl << taskInfo.target_object <<
			"Destination: " << std::endl << taskInfo.destination
		);
        */
		int numOfNonTargetObjects = taskInfo.non_target_objects.size();
		//std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
		//std::cout << "Non-target objects:" << std::endl;
		for(int i=0; i<numOfNonTargetObjects; i++)
		{
			//std::cout << taskInfo.non_target_objects[i] << std::endl;
		}

		int numOfFurniture = taskInfo.furniture.size();
		//std::cout << "Number of furniture: " << numOfFurniture << std::endl;
		//std::cout << "Furniture objects:" << std::endl;
		for(int i=0; i<numOfFurniture; i++)
		{
			//std::cout << taskInfo.furniture[i] << std::endl;
		}

		isTaskInfoReceived = true;
	}

	void avatarStatusMessageCallback(const human_navigation::HumanNaviAvatarStatus::ConstPtr& message)
	{
		avatarStatus = *message;
        /*
		ROS_INFO_STREAM(
			"Subscribe avatar status message:" << std::endl <<
			"Head: " << std::endl << avatarStatus.head <<
			"LeftHand: " << std::endl << avatarStatus.left_hand <<
			"rightHand: " << std::endl << avatarStatus.right_hand <<
			"objctInLeftHand: " << avatarStatus.object_in_left_hand << std::endl <<
			"objectInRightHand: " << avatarStatus.object_in_right_hand << std::endl <<
			"isTargetObjectInLeftHand: " << std::boolalpha << (bool)avatarStatus.is_target_object_in_left_hand << std::endl <<
			"isTargetObjectInRightHand: " << std::boolalpha << (bool)avatarStatus.is_target_object_in_right_hand << std::endl

		);*/

		isSentGetAvatarStatus = false;
	}

	void objectStatusMessageCallback(const human_navigation::HumanNaviObjectStatus::ConstPtr& message)
	{
		objectStatus = *message;
		int numOfNonTargetObjects = taskInfo.non_target_objects.size();
		//std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
		//std::cout << "Non-target objects:" << std::endl;
		for(int i=0; i<numOfNonTargetObjects; i++)
		{
			//std::cout << taskInfo.non_target_objects[i] << std::endl;
		}

		isSentGetObjectStatus = false;
	}

	bool speakGuidanceMessage(ros::Publisher pubHumanNaviMsg, ros::Publisher pubGuidanceMsg, std::string message, int interval = 1)
	{
		if(speechState == SpeechState::Speakable)
		{
			sendGuidanceMessage(pubGuidanceMsg, message, DISPLAY_TYPE_ALL);
			speechState = SpeechState::None;
			return true;
		}
		else if(speechState == SpeechState::None || speechState == SpeechState::Speaking)
		{
			if(timePrevSpeechStateConfirmed.sec + interval < ros::Time::now().sec)
			{
				sendMessage(pubHumanNaviMsg, MSG_GET_SPEECH_STATE);
				timePrevSpeechStateConfirmed = ros::Time::now();
				speechState = SpeechState::WaitingState;
			}
		}

		return false;
	}




    double CalculateRad(double X, double Y)
    {
      double rad;
      rad = atan(Y/X);
      ROS_INFO("X=%lf", X);
      ROS_INFO("Y=%lf", Y);
      ROS_INFO("rad=%lf", rad);
        if(X<0)
        {
          if(rad<0)
            rad = rad + M_PI;
          else if(rad>0)
            rad = rad - M_PI;
        }
      return rad;
    }


    void searchClosestFurniture()
    {
        int numOfFurniture = taskInfo.furniture.size();
        double closestDistance = 1000;
        double X;
        double Y;
        double Z;
        for(int i = 0;i < numOfFurniture; i++)
        {
            X = fabs(taskInfo.target_object.position.x - taskInfo.furniture[i].position.x);
            Y = fabs(taskInfo.target_object.position.y - taskInfo.furniture[i].position.y);
            Z = fabs(taskInfo.target_object.position.z - taskInfo.furniture[i].position.z);
            if(closestDistance > (sqrt(pow(X,2)+pow(Y,2)+pow(Z,2))) )
            {
                closestDistance = (sqrt(pow(X,2)+pow(Y,2)+pow(Z,2)));
                furnitureName = taskInfo.furniture[i].name;
            }
        }
    }


  tf::StampedTransform getTfBase(tf::TransformListener &tf_listener)
  {
    tf::StampedTransform tf_transform;

    try
    {
      tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), tf_transform);
      //tf_listener.lookupTransform("/odom", "/arm_flex_link", ros::Time(0), tf_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    return tf_transform;
  }

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  for(int i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i]=="arm_lift_joint")
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
      return;
    }
    if(joint_state->name[i] == "arm_flex_joint")
    {
      arm_flex_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "wrist_flex_joint")
    {
      wrist_flex_joint_pos_ = joint_state->position[i];
    }
  }
}

void moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist twist;
  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  publisher.publish(twist);
}


void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const int duration_sec)
{
  joint_trajectory.joint_names.push_back("arm_lift_joint");
  joint_trajectory.joint_names.push_back("arm_flex_joint");
  joint_trajectory.joint_names.push_back("arm_roll_joint");
  joint_trajectory.joint_names.push_back("wrist_flex_joint");
  joint_trajectory.joint_names.push_back("wrist_roll_joint");


  arm_joint_point.positions = {position, 0.0, 0.0f, -1.57, 0.0f};

  arm_joint_point.time_from_start = ros::Duration(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point);
  publisher.publish(joint_trajectory);

}

void nearTarget()
{
    double yoko_box[20]={0};
    double tate_box[20]={0};
    int count_yoko = 0;
    int count_tate = 0;
    int number = 0;
    wayMsg = "I'm pointing at";
    for(int i=0; i<taskInfo.non_target_objects.size(); i++)
    {
        if(taskInfo.target_object.name == taskInfo.non_target_objects[i].name)
        {
            if((int)((taskInfo.target_object.position.x * 10)) == (int)((10*taskInfo.non_target_objects[i].position.x)))
            {
                if((int)((taskInfo.target_object.position.z * 10)) == (int)((10*taskInfo.non_target_objects[i].position.z))){
                yoko_box[count_yoko] = taskInfo.non_target_objects[i].position.y;
                count_yoko++;
                }
            }
            else if((int)((taskInfo.target_object.position.y * 10)) == (int)((10*taskInfo.non_target_objects[i].position.y)))
            {
                if((int)((taskInfo.target_object.position.z * 10)) == (int)((10*taskInfo.non_target_objects[i].position.z))){
                tate_box[count_tate] = taskInfo.non_target_objects[i].position.x;
                count_tate++;
                }
            }
        }
     }
     if(count_yoko != 0)
     {
         for(int i=0; i<count_yoko;i++)
         {
            if(taskInfo.target_object.position.y > yoko_box[i])
                number++;
         }
         orderMsg = "the " + orderList[number];
        if(taskInfo.target_object.position.y<0)
            wayMsg = "from the right";
         else if(taskInfo.target_object.position.y >= 0)
            wayMsg = "from the right";
      }

     else if(count_tate != 0)
     {
         for(int i=0; i<count_tate;i++)
         {
            if(fabs(taskInfo.target_object.position.x) > fabs(tate_box[i]))
                number++;
         }
         orderMsg = "the " + orderList[number];
         if(taskInfo.target_object.position.y<0)
            wayMsg = "from the right";
         else if(taskInfo.target_object.position.y >= 0)
            wayMsg = "from the left";
     }
}

void nearDestination()
{
    double tate_box[10]={0};
    double yoko_box[10]={0};
    int count_tate = 0;
    int count_yoko = 0;
    int number = 0;
    wayMsg = "I'm pointing at";
    orderMsg = "that";
    for(int i=0; i<taskInfo.furniture.size(); i++)
    {
        if( ( (int)(taskInfo.destination.position.x * 10) ) == ( (int)(taskInfo.furniture[i].position.x * 10) )
        && ((int)((taskInfo.destination.position.y * 10)) == (int)((taskInfo.furniture[i].position.y * 10 ))) )
        {

            destinationName = taskInfo.furniture[i].name;
            //ROS_INFO("destinationName:%s",destinationName.c_str());
        }
     }
     for(int i=0; i<taskInfo.furniture.size(); i++)
    {
        if(destinationName == taskInfo.furniture[i].name)
        {

            if((int)((taskInfo.destination.position.x * 10)) == (int)((10*taskInfo.furniture[i].position.x)))
            {
                yoko_box[count_yoko] = taskInfo.furniture[i].position.y;
                count_yoko++;
            }
            else if((int)((taskInfo.destination.position.y * 10)) == (int)((10*taskInfo.furniture[i].position.y)))
            {
                tate_box[count_tate] = taskInfo.furniture[i].position.x;
                count_tate++;
            }

        }
     }
     if(count_yoko >1 )
     {
         for(int i=0; i<count_yoko;i++)
         {
            if((int)(taskInfo.destination.position.y*10) > (int)(10*yoko_box[i]))
                number++;
         }
         orderMsg = "the " + orderList[number];

         if(taskInfo.destination.position.y<0)
            wayMsg = "from the right";
         else if(taskInfo.destination.position.y >= 0)
            wayMsg = "from the right";
      }

     else if(count_tate > 1)
     {
         for(int i=0; i<count_tate;i++)
         {
            if(fabs((int)(taskInfo.destination.position.x*10)) > fabs((int)(10*tate_box[i])))
                number++;
         }
         orderMsg = "the " + orderList[number];
          if(taskInfo.destination.position.y<0)
            wayMsg = "from the right";
         else if(taskInfo.destination.position.y >= 0)
            wayMsg = "from the left";
     }
}

void guideTarget(double x, double y,double z)
{
    double kyori_z= taskInfo.target_object.position.z - z;

    if(fabs(kyori_z)>THETA_Z)
    {
        if(kyori_z > 0 &&  fabs(kyori_z)>THETA_Z)
            onWayMsg = "higher";
        else if(kyori_z < 0 &&  fabs(kyori_z)>THETA_Z)
            onWayMsg = "lower";
    }
}

double calculateKyori(double X, double Y)
{
  double x,y, kyori;
  x = fabs(X - avatarStatus.head.position.x);
  y = fabs(Y - avatarStatus.head.position.y);
  kyori = sqrt(pow(x,2)+pow(y,2));
  return kyori;
}

void moveBaseTwist(tf::TransformListener &listener_, ros::Publisher &publisher, double linear_x, double linear_y, double theta, double duration_sec)
{
  if(listener_.canTransform("/odom", "/base_footprint", ros::Time(0)) == false)
  {
    ROS_INFO("not transform data");
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
  publisher.publish(joint_trajectory);
  ROS_INFO("published twist");
}

public:
	int run(int argc, char **argv)
	{
		ros::init(argc, argv, "human_navi_sample");

		ros::NodeHandle nodeHandle;

		ros::Rate loopRate(10);

		init();


		ROS_INFO("Human Navi sample start!");

		ros::Subscriber subHumanNaviMsg = nodeHandle.subscribe<human_navigation::HumanNaviMsg>("/human_navigation/message/to_robot", 100, &HumanNavigationSample::messageCallback, this);
		ros::Subscriber subTaskInfoMsg = nodeHandle.subscribe<human_navigation::HumanNaviTaskInfo>("/human_navigation/message/task_info", 1, &HumanNavigationSample::taskInfoMessageCallback, this);
		ros::Subscriber subAvatarStatusMsg = nodeHandle.subscribe<human_navigation::HumanNaviAvatarStatus>("/human_navigation/message/avatar_status", 1, &HumanNavigationSample::avatarStatusMessageCallback, this);
		ros::Subscriber subObjectStatusMsg = nodeHandle.subscribe<human_navigation::HumanNaviObjectStatus>("/human_navigation/message/object_status", 1, &HumanNavigationSample::objectStatusMessageCallback, this);
		ros::Publisher pubHumanNaviMsg = nodeHandle.advertise<human_navigation::HumanNaviMsg>("/human_navigation/message/to_moderator", 10);
		ros::Publisher pubGuidanceMsg  = nodeHandle.advertise<human_navigation::HumanNaviGuidanceMsg>("/human_navigation/message/guidance_message", 10);

    std::string pub_msg_to_moderator_topic_name;
    std::string sub_joint_state_topic_name;
    std::string pub_base_twist_topic_name;
    std::string pub_arm_trajectory_topic_name;
    std::string pub_gripper_trajectory_topic_name;
    std::string pub_base_trajectory_topic_name;

    nodeHandle.param<std::string>("pub_msg_to_moderator_topic_name",   pub_msg_to_moderator_topic_name,   "/human_navigation/message/to_moderator");

    nodeHandle.param<std::string>("sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/hsrb/joint_states");
    nodeHandle.param<std::string>("pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/command_velocity");
    nodeHandle.param<std::string>("pub_base_trajectory_topic_name",    pub_base_trajectory_topic_name,    "/hsrb/omni_base_controller/command");
    nodeHandle.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
    nodeHandle.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_controller/command");

    ros::Publisher  pub_msg                = nodeHandle.advertise<human_navigation::HumanNaviMsg>(pub_msg_to_moderator_topic_name, 10);

    ros::Subscriber sub_joint_state        = nodeHandle.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &HumanNavigationSample::jointStateCallback, this);
    ros::Publisher  pub_base_twist         = nodeHandle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
    ros::Publisher  pub_arm_trajectory     = nodeHandle.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    ros::Publisher  pub_gripper_trajectory = nodeHandle.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);
    ros::Publisher pub_base_trajectory    = nodeHandle.advertise<trajectory_msgs::JointTrajectory>(pub_base_trajectory_topic_name, 10);
    tf::TransformListener tf_listener;

    std::string arm_lift_joint_name   = "arm_lift_joint";
    std::string arm_flex_joint_name   = "arm_flex_joint";
    std::string wrist_flex_joint_name = "wrist_flex_joint";
    std::string wrist_roll_joint_name = "wrist_roll_joint";


		ros::Time time;
		while (ros::ok())
		{
			switch(step)
			{
				case Initialize:
				{
					reset();
					ROS_INFO("##### Initialized ######");
					step++;
					break;
				}

				case Ready:
				{
					if(isStarted)
					{
						sendMessage(pubHumanNaviMsg, MSG_I_AM_READY);
            sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
            isSentGetObjectStatus = true;
						ROS_INFO("Task start");
            step++;
					}
					break;
				}

				case WaitTaskInfo:
				{
				    tf::StampedTransform tf_transform = getTfBase(tf_listener);
					if(isTaskInfoReceived)
					{
					    rad_target = CalculateRad(taskInfo.target_object.position.x, taskInfo.target_object.position.y);
              rad_destination = CalculateRad(taskInfo.destination.position.x, taskInfo.destination.position.y);
              step++;
					}
					break;
			}

				case Introduction:
				{
          double arm_height = (taskInfo.target_object.position.z - 0.6);
          if(arm_height < 0 )
            arm_height = 0;
		      guideMsg = "Please look back and find a big robot in this room.";
			    if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
			    {
              moveBaseTwist(tf_listener, pub_base_trajectory, 0.0, 0.0, rad_target, 0.5);
              moveArm(pub_arm_trajectory, arm_lift_joint_name, arm_height, 0.5);
			        step++;
              ROS_INFO("WaitPointingTarget");
			    }
				    break;

				}

        case MoveToTargetInstruction:
        {
          guideMsg = "Go straight in the direction that the robot is pointing.";
          if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
          {
              step++;
          }
          break;
        }

        case MoveToTarget:
        {
          double WaitTime = 1.0;
          int digit[3] = {0};
          std::string kyori = "";
          tf::StampedTransform tf_transform = getTfBase(tf_listener);
					if(time.sec + WaitTime < ros::Time::now().sec)
					{
              sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
              //sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
              isSentGetAvatarStatus = true;
              //isSentGetObjectStatus = true;
              kyori_from_avater = calculateKyori(taskInfo.target_object.position.x, taskInfo.target_object.position.y);
              digit[0] = int(kyori_from_avater * 10);
              digit[1] = digit[0]/10;
              digit[2] = digit[0]%10;
              kyori = std::to_string(digit[1]) + "." + std::to_string(digit[2]);
							time = ros::Time::now();
              guideMsg = "It is " + kyori + " meters to target";
              sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
              speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg);

					}
          if(kyori_from_avater < THETA_KYORI)
          {
            step++;
            ROS_INFO("MoveToTarget");
          }
          if(isRequestReceived)
          {
            //guideMsg = "It is " + std::to_string(kyori_from_avater) + " meters to target";
            ROS_INFO("%s", guideMsg.c_str());
              if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
              {
                       isRequestReceived = false;
              }
          }
          break;
        }

        case ReachToTarget:
        {
          guideMsg = "Stop.";
          if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
          {
              step++;
          }
          break;
        }

				case GuideForTakingObject:
				{
            nearTarget();
            targetName = taskInfo.target_object.name;
            std::replace(targetName.begin(), targetName.end(), '_', ' ');
            guideMsg = "Please take " + orderMsg + " " + targetName + " " +
            wayMsg + ".";
				    if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
				    {
				        step++;
				    }
            break;
        }

        case WaitGraspTarget:
        {
          int releaseFlag = 1;
          double WaitTime = 1.0;

          tf::StampedTransform tf_transform = getTfBase(tf_listener);
					if(time.sec + WaitTime < ros::Time::now().sec)
					{
              sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
              //sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
              isSentGetAvatarStatus = true;
              //isSentGetObjectStatus = true;
							time = ros::Time::now();
					}
          if(avatarStatus.is_target_object_in_right_hand || avatarStatus.is_target_object_in_left_hand)
          {
            step++;
          }

          else if((avatarStatus.object_in_left_hand != "") || (avatarStatus.object_in_right_hand != ""))
          {
            if(!(avatarStatus.is_target_object_in_right_hand) && !(avatarStatus.is_target_object_in_left_hand))
            {
              guideMsg = "It is wrong object. Relese it";
              if(releaseFlag == 1)
              {
                if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
                {
                  releaseFlag = 0;
                }
              }

            }
          }
          else if((avatarStatus.object_in_left_hand == "") && (avatarStatus.object_in_right_hand == ""))
          {
            releaseFlag = 1;
          }

          if(isRequestReceived)
					{
            guideMsg = "Please take " + orderMsg + " " + targetName + " " + wayMsg + ".";
						if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
						{
							isRequestReceived = false;
						}
					}

          break;
        }

        case LookMeAgain:
        {

            guideMsg = "Good job! Please look at robot again.";
            if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
            {
              moveBaseTwist(tf_listener, pub_base_trajectory, 0.0, 0.0, ((-rad_target) + rad_destination), 0.5);
              step++;
            }
            break;
        }

        case MoveToDestinationInstruction:
        {
            guideMsg = "Go straight in the direction the robot is pointing.";
            if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
          {
              step++;
              ROS_INFO("MoveToDestination");
          }
              break;
        }

        case MoveToDestination:
        {
          double WaitTime = 0.5;
          int digit[3] = {0};
          std::string kyori = "";
          tf::StampedTransform tf_transform = getTfBase(tf_listener);
					if(time.sec + WaitTime < ros::Time::now().sec)
					{
              sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
              //sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
              isSentGetAvatarStatus = true;
              //isSentGetObjectStatus = true;
              kyori_from_avater = calculateKyori(taskInfo.destination.position.x, taskInfo.destination.position.y);
							time = ros::Time::now();
              digit[0] = int(kyori_from_avater * 10);
              digit[1] = digit[0]/10;
              digit[2] = digit[0]%10;
              kyori = std::to_string(digit[1]) + "." + std::to_string(digit[2]);
              time = ros::Time::now();
              guideMsg = "It is " + kyori + " meters to target";
              sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
              speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg);
					}
          if(kyori_from_avater < THETA_KYORI)
          {
            step++;
            ROS_INFO("ReachToDestination");
          }
          if(isRequestReceived)
          {
            guideMsg = "It is " + kyori + " meters to destination.";
              if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
              {
                       isRequestReceived = false;
               }
          }
          break;
        }

        case ReachToDestination:
        {
          guideMsg = "Stop.";
          if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
          {
              step++;
          }
          break;
        }

        case GuideForDestination:
        {
            nearDestination();
            std::replace(destinationName.begin(), destinationName.end(), '_', ' ');
            guideMsg = "Please put it on or in " + orderMsg + " " + destinationName +" " + wayMsg + ".";
             ROS_INFO("%s",guideMsg.c_str());

			      if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
			      {
			           step++;
			      }
            break;

        }

        case WaitTaskFinished:
        {
		        if(isFinished)
				    {
					         ROS_INFO("Task finished");
					         step++;
					         break;
             }
            double WaitTime = 3.0;

   					if(time.sec + WaitTime < ros::Time::now().sec)
   					{
                 sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
                 //sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
                 isSentGetAvatarStatus = true;
                 //isSentGetObjectStatus = true;
   							time = ros::Time::now();
   					}

		        if(isRequestReceived)
  				  {
  				        guideMsg = "Please put it on " + orderMsg + " " + destinationName +
                            " " + wayMsg + ".";
  					      if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
  					      {
						           isRequestReceived = false;
	                }
  				  }
				    tf::StampedTransform tf_transform = getTfBase(tf_listener);
				    break;
      }

			case TaskFinished:
			{
				// Wait MSG_GO_TO_NEXT_SESSION or MSG_MISSION_COMPLETE
				break;
			}
		}
			ros::spinOnce();
			loopRate.sleep();
		}
		return 0;
    }
};


int main(int argc, char **argv)
{
	HumanNavigationSample humanNaviSample;
	humanNaviSample.run(argc, argv);
};
