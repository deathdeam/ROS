//application node which is used to control the sequence of actionss in our scan & plan task
#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/service_client.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <myworkcell_core/PlanCartesianPath.h>

class ScanNPlan{
private:
//planning componenets
        ros::ServiceClient vision_client_;
        ros::ServiceClient cartesian_client_;
        //To declear ac_, we can only use {} instead of (). Not sure why
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_{"follow_joint_trajectory",true};
public:
    ScanNPlan(ros::NodeHandle& nh){
	vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
	cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
	}
    void start(const std::string& base_frame){
	ROS_INFO("Attempting to localize part");
	
	myworkcell_core::LocalizePart srv;
	srv.request.base_frame = base_frame;
	ROS_INFO_STREAM("Requesting pose in base frame:" << base_frame);
	
	if(!vision_client_.call(srv)){
		ROS_ERROR("Could not localize part");
		return;
	}
	ROS_INFO_STREAM("part localzed: " << srv.response);
	
	geometry_msgs::Pose move_target = srv.response.pose;
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	
	move_group.setPoseReferenceFrame(base_frame);
	move_group.setPoseTarget(move_target);
	move_group.move();
	
	myworkcell_core::PlanCartesianPath cartesian_srv;
	cartesian_srv.request.pose = move_target;
	if(!cartesian_client_.call(cartesian_srv)){
		ROS_ERROR("Could not plan for path");
		return;
	}
	ROS_INFO("Got cart path, executing");
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = cartesian_srv.response.trajectory;
	ac_.sendGoal(goal);
	ac_.waitForResult();
	ROS_INFO("Done");
	}	

};
int main(int argc, char** argv){
	ros::init(argc,argv,"myworkcell_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_node_handle ("~");
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();
	
	std::string base_frame;
	private_node_handle.param<std::string>("base_frame",base_frame,"world");
	
	ScanNPlan app(nh);
	
	ROS_INFO("ScanNPlan node has been initialized");
	ros::Duration(.5).sleep();
	//~ mean the private_node-handle's namespace is /myworkcell_node
	
	
	app.start(base_frame);
	//ros::spin();
	ros::waitForShutdown();

}
