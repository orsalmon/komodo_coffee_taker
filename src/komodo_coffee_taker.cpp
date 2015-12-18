#include <string>
#include <cmath>
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "control_msgs/JointControllerState.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/CollisionObject.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib_msgs/GoalStatus.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base/move_base.h"
#include "komodo_coffee_taker/CoffeeTaker.h"

#define MOVE_BASE_FRAME 		"map"
#define MAX_ELEV_VALUE 			0.5
#define MIN_ELEV_VALUE 			0.0
#define ELEV_ERROR_TOLERANCE 	0.05
#define PI 						3.141592654
#define BASE_SPACE_FROM_OBJECT 	0.6 



/*Prototypes*/
bool coffee_taker(komodo_coffee_taker::CoffeeTaker::Request &req, komodo_coffee_taker::CoffeeTaker::Response &res);
void getGoalandEndPos(komodo_coffee_taker::CoffeeTaker::Request &data); 
void updateGoalPos(double posX,double posY,double posZ,double orientation);
void updateEndPos(double posX,double posY,double posZ,double orientation);
void setMoveBaseCommands(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_client);
void setArmMoveCommands(moveit::planning_interface::MoveGroup &group);
void elevatorStateRecived(const control_msgs::JointControllerState &state);
void moveElevator(double posZ);
bool pickHandler();
bool placeHandler();


/*Global variables*/
double 															currentGoalPosition[3];	
double 															currentEndPosition[3];
double 															currentGoalTheta 					= 0;
double 															currentEndTheta 					= 0;
double 															elevator_current_state_error;
double 															elevator_current_command;
ros::Subscriber 												elevator_state_subscriber;
ros::Publisher 													display_arm_trajectory_publisher;
ros::Publisher 													elevator_command_publisher;
moveit_msgs::DisplayTrajectory 									display_arm_trajectory;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> 	*move_base_client;
tf::TransformListener 											tf_listener;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "komodo_coffee_taker");
	ros::NodeHandle n;

	ros::ServiceServer service 			= n.advertiseService("coffee_taker", coffee_taker);
	display_arm_trajectory_publisher 	= n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	elevator_command_publisher 			= n.advertise<std_msgs::Float64>("/elevator_controller/command", 1);
	elevator_state_subscriber 			= n.subscribe("/elevator_controller/state", 1, &elevatorStateRecived);
	move_base_client 					= new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);

	while(!move_base_client->waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}


	ROS_INFO("Ready for commands:");

  	ros::spin();

	return 0;
}

bool coffee_taker(komodo_coffee_taker::CoffeeTaker::Request  &req,
             	  komodo_coffee_taker::CoffeeTaker::Response &res)
{
	moveit::planning_interface::PlanningSceneInterface scene;
	moveit::planning_interface::MoveGroup group("arm");
	moveit::planning_interface::MoveGroup::Plan my_plan;
	
	res.pick_success.data 	= false;		/*Initialize the response of the service*/
	res.place_success.data 	= false;

	getGoalandEndPos(req);
	moveElevator(req.pick_xyz[2].data);
	ROS_INFO("Waiting for elevator to finish");
	while (elevator_current_state_error > ELEV_ERROR_TOLERANCE)
	{
		ros::Duration(0.1).sleep();
	}

	while (res.pick_success.data != true)
	{
		setMoveBaseCommands(move_base_client);
		move_base_client->waitForResult();
		while (move_base_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			//TODO check where is the problem (orientation/position) and update MoveBaseCommands with another option
		}

		setArmMoveCommands(group);
		while (group.plan(my_plan) != true)
		{
			//TODO check why it's impossible to plan trajectory and give a solution (maybe by move_base action)
			//		and then update MoveIt command
		}
		ROS_INFO("Visualizing arm plan");
		ros::Duration(5.0).sleep(); //Sleep to give Rviz time to visualize the plan
		/* Uncomment below line when working with a real robot*/
		/* group.move() */
		/*if (pickHandler() == true)
			res.pick_success.data == true;*/
	}

	//TODO write almost the same while loop for the place part


	if (res.pick_success.data && res.place_success.data)
		return true;
	else
		return false;
}

/* getGoalPos sets for the first time the goal position and orientation that sent via ROSSERVICE */
void getGoalandEndPos(komodo_coffee_taker::CoffeeTaker::Request &data)
{
	currentGoalPosition[0]	= data.pick_xyz[0].data;
	currentGoalPosition[1]	= data.pick_xyz[1].data;
	currentGoalPosition[2]	= data.pick_xyz[2].data;
	currentGoalTheta		= data.pick_orientation.data * PI / 180.0;

	ROS_INFO("Object position: x=%.4f y=%.4f z=%.4f \nObject orientation: theta=%.4f",currentGoalPosition[0],
																					  currentGoalPosition[1],
																					  currentGoalPosition[2],
																					  currentGoalTheta);


	currentEndPosition[0]	= data.place_xyz[0].data - data.pick_xyz[0].data; 	// End position is the subtraction
	currentEndPosition[1]	= data.place_xyz[1].data - data.pick_xyz[1].data;	// of the place vector with 
	currentEndPosition[2]	= data.place_xyz[2].data - data.pick_xyz[2].data;	// the pick vector
	currentEndTheta			= data.place_orientation.data;

	ROS_INFO("End position: x=%.4f y=%.4f z=%.4f \nEnd orientation: theta=%.4f",currentEndPosition[0],
																				currentEndPosition[1],
																				currentEndPosition[2],
			  								  									currentEndTheta);


	return;
}

/* updateGoalPos updates the current goal position and orientation  */
void updateGoalPos(double posX,double posY,double posZ,double orientation)
{
	currentGoalPosition[0] 	= posX;
	currentGoalPosition[1] 	= posY;
	currentGoalPosition[2] 	= posZ;
	currentGoalTheta		= orientation;

	return;
}

/* updateEndPos updates the current goal position and orientation  */
void updateEndPos(double posX,double posY,double posZ,double orientation)
{
	currentEndPosition[0] 	= posX;
	currentEndPosition[1] 	= posY;
	currentEndPosition[2] 	= posZ;
	currentEndTheta		= orientation;	

	return;
}

/* setMoveBaseCommands sends the current goal to the move_base_client  */
//TODO check the relation between the goal point and the robot frame 
void setMoveBaseCommands(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_client)
{
	double unit_x, unit_y, original_length;
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id 	=	MOVE_BASE_FRAME;
	goal.target_pose.header.stamp 		=	ros::Time::now();

	original_length = sqrt(pow(currentGoalPosition[0],2) + pow(currentGoalPosition[1],2));

	if (original_length != 0) 		//avoid from singularity
	{
		unit_x								= 	currentGoalPosition[0] / original_length;
		unit_y								= 	currentGoalPosition[1] / original_length;
		goal.target_pose.pose.position.x 	=	currentGoalPosition[0] - unit_x * BASE_SPACE_FROM_OBJECT;
		goal.target_pose.pose.position.y 	= 	currentGoalPosition[1] - unit_y * BASE_SPACE_FROM_OBJECT;
	}
	else
	{
		goal.target_pose.pose.position.x 	=	- BASE_SPACE_FROM_OBJECT;
		goal.target_pose.pose.position.y 	= 	- BASE_SPACE_FROM_OBJECT;
	}
	
	goal.target_pose.pose.orientation.w = 	cos(currentGoalTheta / 2);		//Quaternion definition
	goal.target_pose.pose.orientation.z = 	sin(currentGoalTheta / 2);		//Quaternion definition

	ROS_INFO("Sending move_base goal");
	move_base_client->sendGoal(goal);

	return;

}

/* setArmMoveCommands starting moveIt planner (without actually act the komodo arm) */
//TODO check the relation between the pickup and the target_pose that needed
void setArmMoveCommands(moveit::planning_interface::MoveGroup &group)
{
	geometry_msgs::PoseStamped target_pose_map_frame, target_pose_arm_frame;

	target_pose_map_frame.header.frame_id 		= "/map";
	target_pose_map_frame.pose.orientation.x 	= 0.0;
	target_pose_map_frame.pose.orientation.y 	= 0.0;
	target_pose_map_frame.pose.orientation.z 	= 0.0;
	target_pose_map_frame.pose.orientation.w 	= 0.0;
	target_pose_map_frame.pose.position.x 		= currentGoalPosition[0];
	target_pose_map_frame.pose.position.y 		= currentGoalPosition[1];
	target_pose_map_frame.pose.position.z 		= currentGoalPosition[2];

	tf_listener.transformPose("Arm_base_link", target_pose_map_frame, target_pose_arm_frame);

	ROS_INFO("Sending moveIt target to plan trajectory");
	group.setPoseTarget(target_pose_arm_frame.pose);

	return;
}

/* elevatorStateRecived is callback function for elevator_state_subscriber and its update the elevator current state error */
void elevatorStateRecived(const control_msgs::JointControllerState &state)
{
	elevator_current_state_error 	= state.error;
	elevator_current_command 		= state.command;
}

/* moveElevator sending commands to the komodo elevator depanding on the Z position */
void moveElevator(double posZ)
{
	std_msgs::Float64 elevator_command;

	if (posZ >= MAX_ELEV_VALUE)
	{
		elevator_command.data = MAX_ELEV_VALUE;
	}
	else if (posZ <= MIN_ELEV_VALUE)
	{
		elevator_command.data = MIN_ELEV_VALUE;
	}
	else
	{
		elevator_command.data = posZ;
	}

	elevator_command_publisher.publish(elevator_command);
	ROS_INFO("Sending elevator command");
	ros::Duration(1.0).sleep();

	return;
}