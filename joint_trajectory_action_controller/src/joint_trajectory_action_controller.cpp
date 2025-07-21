/**
 * Joint Trajectory Action Controller
 *
 * This Controller translates the produced trajectory produced by MoveIt for the UAV.
 *
 * Updated: 2018/05/10
 * Contact: nihad.s91@gmail.com
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <joint_trajectory_action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

class Controller{
private:
	typedef actionlib::ActionServer<joint_trajectory_action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	Controller(ros::NodeHandle &n) :
		node_(n),
		action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&Controller::goalCB, this, _1),
				boost::bind(&Controller::cancelCB, this, _1),
				false),
				has_active_goal_(false)
{
		created=0;

//		trajectory_pub = node_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
		trajectory_pub = node_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

		action_server_.start();
		ROS_INFO_STREAM("\n Node ready! \n");
}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Publisher trajectory_pub;

	mav_msgs::EigenTrajectoryPoint trajectory_point;
	trajectory_msgs::MultiDOFJointTrajectory desired_wp;

	pthread_t trajectoryExecutor;

	int created;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

  	double current_time; 
  	double start_time; 

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(created){
				ROS_INFO_STREAM("Stop thread \n");
				pthread_cancel(trajectoryExecutor);
				created=0;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(created){
				pthread_cancel(trajectoryExecutor);
				created=0;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;

		//virtual Base
		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			created=1;
			ROS_INFO_STREAM("Thread for trajectory execution created \n");
		} else {
			ROS_INFO_STREAM("Thread creation failed! \n");
		}

	}

	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void executeTrajectory(){
		if(toExecute.joint_names[0]=="virtual_joint" && toExecute.points.size()>0){
			for(int k=0; k<toExecute.points.size(); k++){

				//Load the current transform from the Trajectory
				geometry_msgs::Transform_<std::allocator<void> > transform=toExecute.points[k].transforms[0];

				trajectory_msgs::MultiDOFJointTrajectoryPoint point=toExecute.points[k];
				
				//Convert Tranform into EigenTrajectoryPoint
				mav_msgs::eigenTrajectoryPointFromMsg(point, &trajectory_point);

				//Convert EigenTrajectoryPoint into Trajectory Message
				mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &desired_wp);

				//Publish the Trajectory Message
				if(k==0){
					start_time = ros::Time::now().toSec();
					current_time = ros::Time::now().toSec();

					desired_wp.header.stamp = ros::Time::now();
        				desired_wp.header.frame_id = "trajectory_tranform";
					trajectory_pub.publish(desired_wp);
				}
				else{

					while( (current_time-start_time) < toExecute.points[k].time_from_start.toSec() ){
					current_time = ros::Time::now().toSec();
					}
					desired_wp.header.stamp = ros::Time::now();
        				desired_wp.header.frame_id = "trajectory_transform";
					trajectory_pub.publish(desired_wp);
				}
			}
		}
		active_goal_.setSucceeded();
		has_active_goal_=false;
		created=0;

	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_controller_node");
	ros::NodeHandle node;
	Controller control(node);

	ros::spin();

	return 0;
}
