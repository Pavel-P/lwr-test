// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

//message types for gripper
#include <robotiq_s_model_articulated_msgs/SModelRobotInput.h>
#include <robotiq_s_model_articulated_msgs/SModelRobotOutput.h>

//subscriber msgs
#include <std_msgs/String.h>
//gazebo msgs
/*#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>*/
#include <gazebo_msgs/ModelStates.h>

#include <stdio.h>		// stdin, stderr 
#include <stddef.h>		// NULL, sizeof, size_t 
#include <stdlib.h>		// atoi, atof 
#include <ctype.h>		// isspace
#include <math.h>
#include <unistd.h>     //usleep
#include <iostream>      //cout

/*#include <ulapi.h>*/

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     const std::vector<double>& time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  int i = 0;
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it, ++i)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay[i];

    result.points.push_back(pt);
  }

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
//  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("lwr/joint_trajectory_controller/follow_joint_trajectory", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  
  ac.sendGoal(goal);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}



class Pose_Listener
{
    public:
        float x, y, z;
        float r_x, r_y, r_z, r_w;
        float t_x, t_y, t_z;
        std::string name;
        std::string target;
        void callback(const gazebo_msgs::ModelStatesConstPtr& msg);
};
void Pose_Listener::callback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
    for (int i = 0; !(msg->name[i].empty()) ;  i++)
    {
        if (msg->name[i].compare(this->name) == 0)
        {
            this->x = msg->pose[i].position.x;
            this->y = msg->pose[i].position.y;
            this->z = msg->pose[i].position.z;
            this->r_w = msg->pose[i].orientation.w;
            this->r_x = msg->pose[i].orientation.x;
            this->r_y = msg->pose[i].orientation.y;
            this->r_z = msg->pose[i].orientation.z;
            break;
        }
    }
    for (int i = 0; !(msg->name[i].empty()) ;  i++)
    {
//        puts("this is happening");
//        std::cout << this->target << std::endl;
        if (msg->name[i].compare(this->target) == 0)
        {
//            puts("the other thing is happening");
            this->t_x = msg->pose[i].position.x;
            this->t_y = msg->pose[i].position.y;
            this->t_z = msg->pose[i].position.z;
            break;
        }
    }
    return;
}

class Grip_Listener
{
    public:
        char status;
        void callback(const robotiq_s_model_articulated_msgs::SModelRobotInputConstPtr& msg);
};
void Grip_Listener::callback(const robotiq_s_model_articulated_msgs::SModelRobotInputConstPtr& msg)
{
    this->status = msg->gSTA;
}

int main (int argc, char **argv)
{
    const float Z_COR = 0.245, Y_COR = 0, X_COR = 0.00, STANDBY_HEIGHT = 0.7;
    const char GRIP_POS = 205;

    ros::init(argc, argv, "demo");
    ros::NodeHandle nh; 

    ros::AsyncSpinner spinner (1);
    spinner.start();
    
    /* Gazebo object positions */
    Pose_Listener gz_listen;
    gz_listen.name.assign("metal_peg");
    gz_listen.target.assign("washer_0");
    ros::Subscriber gz_sub = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1, &Pose_Listener::callback, &gz_listen);

    /* Gripper commands*/
    ros::Publisher robotiq_control = nh.advertise<robotiq_s_model_articulated_msgs::SModelRobotOutput>("left_hand/command", 1000);
    ros::Rate loop_rate(100);

    robotiq_s_model_articulated_msgs::SModelRobotOutput command;
    command.rACT = 1;
    command.rGTO = 1;
    command.rMOD = 1;
    command.rPRA = 0;
    for (int m = 0; m < 10; m++)
    {
        robotiq_control.publish(command);
        loop_rate.sleep();
    }

    /* Gripper state */
    Grip_Listener grip_listen;
    ros::Subscriber grip_sub = nh.subscribe<robotiq_s_model_articulated_msgs::SModelRobotInput>("left_hand/state", 1, &Grip_Listener::callback, &grip_listen);


    /* Descartes Model Initialization */
    
    // Name of description on parameter server. Typically just "robot_description".
    const std::string robot_description = "robot_description";
    // name of the kinematic group you defined when running MoveitSetupAssistant
    const std::string group_name = "full_lwr";
    // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
    const std::string world_frame = "world";
    // tool center point frame (name of link associated with tool)
    const std::string tcp_frame = "lwr_7_link";

    descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
        ROS_INFO("Could not initialize robot model");
        exit(-1);
    }
    else
    {
        model->setCheckCollisions(false);
        ROS_INFO("Robot initialized successfully!");
    }
    // 3. Create a planner and initialize it with our robot model
    descartes_planner::DensePlanner planner;
    planner.initialize(model);


    TrajectoryVec points;
    TrajectoryVec result;
    std::vector<double> joints;
    std::vector<double> times;

    // Get Joint Names
    std::vector<std::string> j_names;
    nh.getParam("controller_joint_names", j_names);
    while (true){
        {
            if ( gz_listen.target.compare("washer_0") == 0 ) {
                gz_listen.target.assign("washer_1");
            } else {
                ROS_WARN ("its not washer_0");
                gz_listen.target.assign("washer_0");
            }
            ROS_WARN("before: %f\n",gz_listen.t_x);
            usleep(1000*1000);
            ROS_WARN("after: %f\n",gz_listen.t_x);


            using namespace descartes_core;
            using namespace descartes_trajectory;
            //poses
            /*TrajectoryPtPtr ps_pt;
            Eigen::Affine3d pose = Eigen::Translation3d(gz_listen.x, gz_listen.y, 0.8) *
            Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
            ps_pt = TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose) ) );

            TrajectoryPtPtr pr_pt;
            pose = Eigen::Translation3d(gz_listen.x + X_COR, gz_listen.y + Y_COR, gz_listen.z + Z_COR) *
            Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
            pr_pt = TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose) ) );

            TrajectoryPtPtr ts_pt;
            pose = Eigen::Translation3d(gz_listen.t_x, gz_listen.t_y, 0.8) *
            Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
            ts_pt = TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose) ) );

            TrajectoryPtPtr tr_pt;
            pose = Eigen::Translation3d(gz_listen.t_x, gz_listen.t_y, gz_listen.t_z + Z_COR) *
            Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
            Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
            tr_pt = TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose) ) );*/

            //pickup standby pos
            TrajectoryPtPtr ps_pt;
            ps_pt = TrajectoryPtPtr( new AxialSymmetricPt(gz_listen.x, gz_listen.y, STANDBY_HEIGHT, M_PI, 0, 0, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );
            //pickup ready pos
            TrajectoryPtPtr pr_pt;
            pr_pt = TrajectoryPtPtr( new AxialSymmetricPt(gz_listen.x + X_COR, gz_listen.y + Y_COR, gz_listen.z + Z_COR, M_PI, 0, 0, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );
            //target standby pos
            TrajectoryPtPtr ts_pt;
            ts_pt = TrajectoryPtPtr( new AxialSymmetricPt(gz_listen.t_x, gz_listen.t_y, STANDBY_HEIGHT, M_PI, 0, 0, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );
            //target ready pos
            TrajectoryPtPtr tr_pt;
            tr_pt = TrajectoryPtPtr( new AxialSymmetricPt(gz_listen.t_x + X_COR, gz_listen.t_y + Y_COR, gz_listen.t_z + Z_COR, M_PI, 0, 0, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );

            //add dummy point if this is the start of execution
            if (points.empty())
            {
                TrajectoryPtPtr dummy_pt;
                for (int q = 0; q < 7; q++) joints.push_back(0);
                dummy_pt = TrajectoryPtPtr( new JointTrajectoryPt(joints) );
                points.push_back(dummy_pt);
            }

            points.push_back(ps_pt);
            points.push_back(pr_pt);
            times.push_back(4);
            times.push_back(5);
            //execute 
            // 4. Feed the trajectory to the planner
            if (!planner.planPath(points))
            {
                ROS_ERROR("Could not solve for a valid path");
                points.clear();
                times.clear();
                break;
            }
            if (!planner.getPath(result))
            {
                ROS_ERROR("Could not retrieve path");
                points.clear();
                times.clear();
                break;
            }
            // 5. Translate the result into a type that ROS understands
            trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, j_names, times);
            // 6. Send the ROS trajectory to the robot for execution
            if (!executeTrajectory(joint_solution))
            {
                ROS_ERROR("Could not execute trajectory!");
                points.clear();
                times.clear();
                break;
            }
            else 
            {
                //set point vector to only contain the last point in the trajectory
                points.clear();
                times.clear();
                points.push_back( result.back() );
                result.clear();
            }

            //grasp gripper
            command.rPRA = GRIP_POS;
            for (int m = 0; m < 10; m++)
            {
                robotiq_control.publish(command);
                loop_rate.sleep();
            }
            usleep(100000); //sleep for 100ms to skip garbage values
            /*while(grip_listen.status == 0);
            //switch targets
            if (grip_listen.status == 2 || grip_listen.status == 1) {
                if ( gz_listen.target.compare("washer_0") ) {
                    gz_listen.target.assign("washer_1");
                } else {
                    gz_listen.target.assign("washer_0");
                }
            } else {
                //release gripper
                command.rPRA = 0;
                for (int m = 0; m < 10; m++)
                {
                    robotiq_control.publish(command);
                    loop_rate.sleep();
                }
            }*/

            points.push_back(ps_pt);
            points.push_back(ts_pt);
            points.push_back(tr_pt);
            times.push_back(5);
            times.push_back(8);
            times.push_back(5);
            //execute
            // 4. Feed the trajectory to the planner
            if (!planner.planPath(points))
            {
                ROS_ERROR("Could not solve for a valid path");
                points.clear();
                times.clear();
                break;
            }
            if (!planner.getPath(result))
            {
                ROS_ERROR("Could not retrieve path");
                points.clear();
                times.clear();
                break;
            }
            // 5. Translate the result into a type that ROS understands
            joint_solution = toROSJointTrajectory(result, *model, j_names, times);
            // 6. Send the ROS trajectory to the robot for execution
            if (!executeTrajectory(joint_solution))
            {
                ROS_ERROR("Could not execute trajectory!");
                points.clear();
                times.clear();
                break;
            }
            else 
            {
                //set point vector to only contain the last point in the trajectory
                points.clear();
                times.clear();
                points.push_back( result.back() );
                result.clear();
            }

            //precise placement
            /* options:
            1. small change, make sure its closer, repeat, still need to calc quaternion to angle
            2. add virtual link at grasp point, get position of link relative to world frame, put link in position of (hole + dist. from grip to object). get angle of peg and rotate cart gripper point to compensate. Convert from quaternion once.

            2 is better.
            */

            //open gripper
            command.rPRA = 0;
            for (int m = 0; m < 10; m++)
            {
                robotiq_control.publish(command);
                loop_rate.sleep();
            }

            points.push_back(ts_pt);
            times.push_back(5);
            //execute
            // 4. Feed the trajectory to the planner
            if (!planner.planPath(points))
            {
                ROS_ERROR("Could not solve for a valid path");
                points.clear();
                times.clear();
                break;
            }
            if (!planner.getPath(result))
            {
                ROS_ERROR("Could not retrieve path");
                points.clear();
                times.clear();
                break;
            }
            // 5. Translate the result into a type that ROS understands
            joint_solution = toROSJointTrajectory(result, *model, j_names, times);
            // 6. Send the ROS trajectory to the robot for execution
            if (!executeTrajectory(joint_solution))
            {
                ROS_ERROR("Could not execute trajectory!");
                points.clear();
                times.clear();
                break;
            }
            else 
            {
                //set point vector to only contain the last point in the trajectory
                points.clear();
                times.clear();
                points.push_back( result.back() );
                result.clear();
            }
            ROS_WARN("GOT_HERE");
        }
        //printf("I say: %f\n", gz_listen.x);
        //printf("I say: %0X\n", grip_listen.status);

    }
}







