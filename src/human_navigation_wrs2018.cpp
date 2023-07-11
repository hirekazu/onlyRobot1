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
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_listener.h>

class HumanNavigationSample
{

private:
        static const char KEYCODE_0 = 0x30;
        static const char KEYCODE_1 = 0x31;
        static const char KEYCODE_2 = 0x32;
        static const char KEYCODE_3 = 0x33;
        static const char KEYCODE_9 = 0x39;

        static const char KEYCODE_UP    = 0x41;
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
        static const char KEYCODE_S = 0x73;
        static const char KEYCODE_U = 0x75;
        static const char KEYCODE_Y = 0x79;
        static const char KEYCODE_Z = 0x7a;

        static const char KEYCODE_COMMA  = 0x2c;
        static const char KEYCODE_PERIOD = 0x2e;

        const std::string ARM_LIFT_JOINT_NAME = "arm_lift_joint";
        const std::string wrist_roll_joint = "wrist_roll_joint";

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
		Introduction1,
		Introduction2,
		Introduction3,
		WaitPointingTarget,
		GuideForTakingObject,
		WaitGraspTarget,
		LookMe2,
        BackHome,
		WaitPointingDuration,
		GuideForDuration,
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

    double rad;
    double way;
    double sita;

    double r,p,y;
    double arm_lift_joint_pos2_;
    double arm_lift_joint_pos1_;
    bool finishMoveArm;
    bool finishRotate;
    bool UP;
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

    const double THETA = 1.0;

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
		way = 0;
        rad = 0;
        r=p=y=0;
        arm_lift_joint_pos1_=0.0;
        arm_lift_joint_pos2_=0.0;
        finishMoveArm = false;
        finishRotate = false;
        UP = false;
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
        /*
		ROS_INFO_STREAM(
			"Subscribe object status message:" << std::endl <<
			"Target object: " << std::endl << taskInfo.target_object
		);
        */
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

    int canReceive( int fd )
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


    void CalculateRad(double X, double Y)
    {
        rad = atan2(Y, X);
        if(rad > 0 )
        {
            way = 1.0;
        }
        else
        {
            way = -1.0;
        }
    }


    double calculate_kyori()
    {
        double X = fabs(taskInfo.target_object.position.x - objectStatus.target_object.position.x);
        double Y = fabs(taskInfo.target_object.position.y - objectStatus.target_object.position.y);
        double Z = fabs(taskInfo.target_object.position.z - objectStatus.target_object.position.z);
        return (sqrt(pow(X,2)+pow(Y,2)+pow(Z,2)));
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
    //ROS_INFO("%s",joint_state->name[i].c_str());
    if(joint_state->name[i]==ARM_LIFT_JOINT_NAME)
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
      //ROS_INFO("lift:%lf",arm_lift_joint_pos2_);

      return;
    }
  }
}

void moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  publisher.publish(twist);
}



void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const int duration_sec)
{

  names.push_back(name);

  positions.push_back(position);
  ros::Duration duration;
  duration.sec = duration_sec;


  joint_trajectory.points.push_back(arm_joint_point);
  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;
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
    /*
    ROS_INFO_STREAM(
    "target_data:" << std::endl <<
    "count_tate: " << count_tate << std::endl <<
    "count_yoko: " << count_yoko << std::endl <<
    "number: " << std::endl << number

    );*/
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
        else
        {
        /*
                ROS_INFO_STREAM(
    "atai:" << std::endl <<
    "dx: " << (int)(taskInfo.destination.position.x * 10) << std::endl <<
    "fx: " << (int)(taskInfo.furniture[i].position.x * 10) << std::endl <<
    "dy: " << (int)(taskInfo.destination.position.y * 10) << std::endl <<
    "fy: " << (int)(taskInfo.furniture[i].position.y * 10 ) << std::endl <<
    "r: " << std::endl << number
    );*/

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
     /*
    ROS_INFO_STREAM(
    "data:" << std::endl <<
    "count_tate: " << count_tate << std::endl <<
    "count_yoko: " << count_yoko << std::endl <<
    "number: " << std::endl << number

    );*/

}

void guideTarget(double x, double y,double z)
{


    //double kyori_x= taskInfo.target_object.position.x - x;
    //double kyori_y= taskInfo.target_object.position.y - y;
    double kyori_z= taskInfo.target_object.position.z - z;

    if(fabs(kyori_z)>THETA)
    {
        if(kyori_z > 0 &&  fabs(kyori_z)>THETA)
            onWayMsg = "higher";
        else if(kyori_z < 0 &&  fabs(kyori_z)>THETA)
            onWayMsg = "lower";
    }
}




public:
	int run(int argc, char **argv)
	{
		ros::init(argc, argv, "human_navi_sample");

		ros::NodeHandle nodeHandle;

		ros::Rate loopRate(10);

		init();

        /////////////////////////////////////////////
        // get the console in raw mode
        char c;
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
        std::string arm_lift_joint_name   = "arm_lift_joint";

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

          nodeHandle.param<std::string>("pub_msg_to_moderator_topic_name",   pub_msg_to_moderator_topic_name,   "/human_navigation/message/to_moderator");

          nodeHandle.param<std::string>("sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/hsrb/joint_states");
          nodeHandle.param<std::string>("pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/opt_command_velocity");
          nodeHandle.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
          nodeHandle.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_trajectory_controller/command");

          ros::Publisher  pub_msg                = nodeHandle.advertise<human_navigation::HumanNaviMsg>(pub_msg_to_moderator_topic_name, 10);

          ros::Subscriber sub_joint_state        = nodeHandle.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &HumanNavigationSample::jointStateCallback, this);
          ros::Publisher  pub_base_twist         = nodeHandle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
          ros::Publisher  pub_arm_trajectory     = nodeHandle.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
          ros::Publisher  pub_gripper_trajectory = nodeHandle.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);
          tf::TransformListener tf_listener;


		ros::Time time;
        //int intro_count = 0;
        //double rad;
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
						step++;

						sendMessage(pubHumanNaviMsg, MSG_I_AM_READY);
                        sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
                        isSentGetObjectStatus = true;
						ROS_INFO("Task start");
                        //ROS_INFO("%s",taskInfo.target_object.name.c_str());
					}
					break;
				}
				case WaitTaskInfo:
				{
				    tf::StampedTransform tf_transform = getTfBase(tf_listener);
					if(isTaskInfoReceived)
					{
					    tf::StampedTransform tf_transform = getTfBase(tf_listener);
					    CalculateRad(taskInfo.target_object.position.x - tf_transform.getOrigin().x(), taskInfo.target_object.position.y - tf_transform.getOrigin().y());
					    if(arm_lift_joint_pos2_ < (taskInfo.target_object.position.z - 0.6) )
					    {
					       UP = true;
					    }
					    else
					    {
					       UP = false;
					    }
                        //step++;
                        step=step+4;
					}
					break;
				}

				case Introduction1:
				{
				    guideMsg = "Hi, I'm robot. Please look for me.";
				    if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
				    {
				        step++;
				    }
				    break;

				}

                case Introduction2:
				{
				    guideMsg = "This is a game that carries things according to my instructions.";
				    if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
				    {
				        step++;
				    }
                    break;
				}
				case Introduction3:
				{

				    guideMsg = "The target may be hidden inside something. Please pay attention to the height of my arm and the pointing direction.";
				    if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
                    {
                        step++;
                    }
                    break;
				}

				case WaitPointingTarget:
				{
				    tf::StampedTransform tf_transform = getTfBase(tf_listener);
                    if(finishRotate == false)
				    {
				        tf::Matrix3x3(tf_transform.getRotation()).getRPY(r, p, y);
				    }


		            if(UP == true && finishMoveArm == false)
		            {
                        if(arm_lift_joint_pos2_> (taskInfo.target_object.position.z - 0.6) )
                        {
                            moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
                            finishMoveArm = true;
                        }

                        else
                        {
                            if(arm_lift_joint_pos1_ >0.67)
                            {
                                moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
                                finishMoveArm = true;
                            }
                            moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.69, 0.1);
                        }
		            }

		            else if (UP == false && finishMoveArm == false)
		            {
                        if( arm_lift_joint_pos2_< (taskInfo.destination.position.z - 0.7))
		                {
		                    moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
		                    finishMoveArm = true;
		                }
		                else
                        {
                            if(arm_lift_joint_pos1_ <0.02)
                            {
                                moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
                                finishMoveArm = true;
                            }
                        }
                            moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.0, 1);

		            }


                    if((fabs(rad - y)) > 0.1  && finishRotate == false)
                    {
                        moveBase(pub_base_twist, 0.0, 0.0, way*0.25);
                    }
                    else
                    {
                        moveBase(pub_base_twist, 0.0, 0.0, 0.0);
                        finishRotate = true;
                    }
                    if(finishMoveArm == true && finishRotate == true)
                    {
                        finishMoveArm = false;
                        finishRotate = false;
                        step++;
                    }

                    break;
				}

				case GuideForTakingObject:
				{


                    nearTarget();
                    targetName = taskInfo.target_object.name;
                    std::replace(targetName.begin(), targetName.end(), '_', ' ');
                    guideMsg = "Let's begin! Please take " + orderMsg + " " + targetName + " " +
                    wayMsg + ".";
                    /*
                    searchClosestFurniture();
                    std::replace(furnitureName.begin(), furnitureName.end(), '_', ' ');
                    guideMsg = "Walk towards the " + furnitureName +". Pick up that "+ orderMsg + " " + targetName +
                        " " + wayMsg +".";
                    */
				    if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
				    {
				        step++;
				    }
                    break;
                }
                case WaitGraspTarget:
                {
                    if(isRequestReceived)
					{
                        guideMsg = "Please take " + orderMsg + " " + targetName + " " +
                            wayMsg + ".";
						if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
						{
							isRequestReceived = false;
						}
					}

                    double WaitTime = 1.0;
                    bool flag = true;

                    tf::StampedTransform tf_transform = getTfBase(tf_listener);
					if(time.sec + WaitTime < ros::Time::now().sec)
					{
                        sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
                        sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
                        isSentGetAvatarStatus = true;
                        isSentGetObjectStatus = true;
						//if(!isSentGetAvatarStatus && !isSentGetObjectStatus)
						//{
							time = ros::Time::now();
                            if( (avatarStatus.object_in_left_hand == "") && (avatarStatus.object_in_right_hand == "") )
                            {
			           		    flag = true;
					        }
						//}
					}
                    //grasp target
					if( (avatarStatus.object_in_left_hand == taskInfo.target_object.name || avatarStatus.object_in_right_hand == taskInfo.target_object.name)
					    && calculate_kyori()>0.03 )
					{
					    step++;
					}
					//grasp same_name with left hand
                    else if( avatarStatus.object_in_left_hand !=""
					    && calculate_kyori()<0.03 )
					{

					    if(flag == true)
					    {

					        guideTarget(avatarStatus.left_hand.position.x,avatarStatus.left_hand.position.y ,avatarStatus.left_hand.position.z);
					        if(wrong_counter < 2){
					            guideMsg = "Please pay attention to the height of my arm and pointing direction! Please take other " + targetName + " " + onWayMsg + ".";
                            }
					        else{
			                    guideMsg = "It may be a trick question. Try looking the back and front of the room. Please pay attention to the height of my arm and pointing direction!";
					        }
                            if(!isSentGetAvatarStatus && !isSentGetObjectStatus){
                                 if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
                                 {
                                    wrong_counter++;
                                    flag = false;
                                 }
                            }
                        }

					}
					//grasp same_name with right hand
                    else if( avatarStatus.object_in_right_hand != ""
					    && calculate_kyori()<0.03 )
					{
					    if(flag == true)
					    {
                            guideTarget(avatarStatus.right_hand.position.x,avatarStatus.right_hand.position.y ,avatarStatus.right_hand.position.z);
					        if(wrong_counter < 2){
					            guideMsg = "Please pay attention to the height of my arm and pointing direction! Please take other " + targetName + " " + onWayMsg + ".";
                            }
					        else{
			                    guideMsg = "It may be a trick question. Try looking the back and front of the room. Please pay attention to the height of my arm and pointing direction!";
					        }
                            if(!isSentGetAvatarStatus && !isSentGetObjectStatus){
                                if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
                                {
                                    wrong_counter++;
                                    flag = false;
                                }
                            }
                        }

					}


					break;
                }

                case LookMe2:
                {
                    guideMsg = "Good job! Please look at me again.";
				    if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
                    {
                        step++;
                    }
                    break;
                }

                case BackHome:
                {
                     tf::StampedTransform tf_transform = getTfBase(tf_listener);
                     tf::Matrix3x3(tf_transform.getRotation()).getRPY(r, p, y);
                    moveBase(pub_base_twist, 0.0, 0.0, -0.5*(way));
                    if(fabs(y)<0.04)
                    {
                        moveBase(pub_base_twist, 0.0, 0.0, 0.0);
                        CalculateRad(taskInfo.destination.position.x - tf_transform.getOrigin().x(), taskInfo.destination.position.y - tf_transform.getOrigin().y());

                        if(arm_lift_joint_pos2_< (taskInfo.destination.position.z - 0.5) )
                        {
                            UP = true;
                        }
                        else
                        {
                            UP = false;
                        }
                        step++;
                    }
                    break;
                }
                case WaitPointingDuration:
                {
				    tf::StampedTransform tf_transform = getTfBase(tf_listener);
				    if(finishRotate == false)
				    {
				        tf::Matrix3x3(tf_transform.getRotation()).getRPY(r, p, y);
				    }
		            if(UP == true && finishMoveArm == false)
		            {
                        if(arm_lift_joint_pos2_> (taskInfo.destination.position.z - 0.6) )
                        {
                            moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
                            finishMoveArm = true;
                        }
                        else
                        {
                            if(arm_lift_joint_pos1_ >0.67)
                            {
                                moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
                                finishMoveArm = true;
                            }
                            moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.69, 0.1);
                        }
		            }
		            else if (UP == false && finishMoveArm == false)
		            {
		                if( arm_lift_joint_pos2_< (taskInfo.destination.position.z - 0.7))
		                {
		                    moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
		                    finishMoveArm = true;
		                }
		                else
                        {
                            if(arm_lift_joint_pos1_ <0.02)
                            {
                                moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
                                finishMoveArm = true;
                            }

                            moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.0, 1);
                        }
		            }

                    if((fabs(rad - y)) > 0.1  && finishRotate == false)
                    {
                        moveBase(pub_base_twist, 0.0, 0.0, way*0.25);
                    }
                    else
                    {
                        moveBase(pub_base_twist, 0.0, 0.0, 0.0);
                        finishRotate = true;
                    }
                    if(finishMoveArm == true && finishRotate == true)
                    {
                        finishMoveArm = false;
                        finishRotate = false;
                        step++;
                    }
                    break;
                }
                case GuideForDuration:
                {


                    nearDestination();
                    std::replace(destinationName.begin(), destinationName.end(), '_', ' ');
                    guideMsg = "Please put it on " + orderMsg + " " + destinationName +
                     " " + wayMsg + ".";
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
