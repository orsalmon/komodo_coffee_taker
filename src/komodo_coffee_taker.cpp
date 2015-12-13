#include <string>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
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

#define BASE_FRAME_ID 	"map"
#define MAX_ELEV_VALUE 	0.5
#define MIN_ELEV_VALUE 	0.0


/*Prototypes*/
bool coffee_taker(komodo_coffee_taker::CoffeeTaker::Request &req, komodo_coffee_taker::CoffeeTaker::Response &res);
void getGoalandEndPos(komodo_coffee_taker::CoffeeTaker::Request &data); 
void updateGoalPos(double posX,double posY,double posZ,double orientation);
void updateEndPos(double posX,double posY,double posZ,double orientation);
void setMoveBaseCommands(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_client);
void setArmMoveCommands(moveit::planning_interface::MoveGroup &group);
void moveElevator(double posZ);
bool pickHandler();
bool placeHandler();


/*Global variables*/
double 															currentGoalPosition[3];	
double 															currentEndPosition[3];
double 															currentGoalTheta 					= 0;
double 															currentEndTheta 					= 0;
ros::Publisher 													display_arm_trajectory_publisher;
ros::Publisher 													elevator_command_publisher;
moveit_msgs::DisplayTrajectory 									display_arm_trajectory;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> 	*move_base_client;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "komodo_coffee_taker");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("coffee_taker", coffee_taker);
	display_arm_trajectory_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	elevator_command_publisher = n.advertise<std_msgs::Float64>("/elevator_controller/command", 1);
	move_base_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);

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
		sleep(10.0); //Sleep to give Rviz time to visualize the plan
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
	currentGoalTheta		= data.pick_orientation.data;

	ROS_INFO("Object position: x=%.4f y=%.4f z=%.4f \nObject orientation: theta=%.4f",currentGoalPosition[0],
																					  currentGoalPosition[1],
																					  currentGoalPosition[2],
																					  currentGoalTheta);


	currentEndPosition[0]	= data.place_xyz[0].data;
	currentEndPosition[1]	= data.place_xyz[1].data;
	currentEndPosition[2]	= data.place_xyz[2].data;
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
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id 	=	BASE_FRAME_ID;
	goal.target_pose.header.stamp 		=	ros::Time::now();
	goal.target_pose.pose.position.x 	=	currentGoalPosition[0];
	goal.target_pose.pose.position.y 	= 	currentGoalPosition[1];
	goal.target_pose.pose.orientation.w = 	currentGoalTheta;

	ROS_INFO("Sending move_base goal");
	move_base_client->sendGoal(goal);

	return;

}

/* setArmMoveCommands starting moveIt planner (without actually act the komodo arm) */
//TODO check the relation between the pickup and the target_pose that needed
void setArmMoveCommands(moveit::planning_interface::MoveGroup &group)
{
	geometry_msgs::Pose target_pose;

	target_pose.orientation.w 	= 0.0;
	target_pose.position.x 		= currentGoalPosition[0];
	target_pose.position.y 		= currentGoalPosition[1];
	target_pose.position.z 		= currentGoalPosition[2];

	ROS_INFO("Sending moveIt target to plan trajectory");
	group.setPoseTarget(target_pose);

	return;
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
	
	return;
}