#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/CollisionObject.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "moveit_test");
	ros::NodeHandle n;
	
	moveit::planning_interface::MoveGroup group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("End effector frame: %s", group.getEndEffectorLink().c_str());
	ros::WallDuration(5.0).sleep();
	
	geometry_msgs::PoseStamped target_pose1;
	target_pose1.header.frame_id = "/dummy_link";
	target_pose1.pose.orientation.w = 1.0;
	target_pose1.pose.orientation.x = 0.0;
	target_pose1.pose.orientation.y = 0.0;
	target_pose1.pose.orientation.z = 0.0;
	target_pose1.pose.position.x = 0.5;
	target_pose1.pose.position.y = 0.0;
	target_pose1.pose.position.z = 0.5;
	group.setPoseTarget(target_pose1,"wrist_link");
	group.setGoalTolerance(0.31);
	//group.setPlanningTime(15.0);
	ROS_INFO("YAAA");
	group.move();
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);






	ros::spin();
	return 0;
}
