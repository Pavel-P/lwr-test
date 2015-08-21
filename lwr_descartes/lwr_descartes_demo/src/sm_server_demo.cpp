/* Simple message server that passes cartesian trajectory points to descartes */

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
#include <descartes_planner/sparse_planner.h>

#include <stdio.h>		// stdin, stderr 
#include <stddef.h>		// NULL, sizeof, size_t 
#include <stdlib.h>		// atoi, atof 
#include <ctype.h>		// isspace
#include <math.h>

#include <ulapi.h>

#include "simple_message_defs.h"

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

struct process_request_thread_args
{
    int id;
    void *thread;

    static descartes_core::RobotModelPtr model;
    static descartes_planner::SparsePlanner planner;
    static std::vector<std::string> j_names;
};

static void process_request_thread_code(process_request_thread_args *args)
{
    enum {INBUF_LEN = 1024};
    char inbuf[INBUF_LEN];
    char *ptr;
    int nchars;
    int nleft;
    int length;
    int message_type;

    cart_traj_pt_request_message ctreq;
    cart_traj_pt_reply_message ctrep;
    joint_traj_pt_request_message jtreq;
    joint_traj_pt_reply_message jtrep;
    joint_info jinfo;

    float x, y, z, rx, ry, rz, t;
    
    bool execute = true;
    bool successful_execution = false;
    bool done = false;
    TrajectoryVec points;
    TrajectoryVec result;
    std::vector<double> times;
    std::vector<double> joints;


    while (!done)
    {
        //get string from socket
        nchars = ulapi_socket_read (args->id, inbuf, sizeof(inbuf));
        if (nchars <= 0)
        {
            ROS_ERROR("No message received");
            break;
        }
        inbuf[sizeof(inbuf)-1] = 0;
        ptr=inbuf;
        nleft = nchars;

        while (nleft > 0)
        {
            //get length and type of message
            memcpy(&length, ptr, sizeof(length));
            memcpy(&message_type, ptr + sizeof(length), sizeof(message_type));
            //handle message types
            switch (message_type)
            {
                case MESSAGE_CART_TRAJ_PT:
                    ctreq.read_cart_traj_pt_request(ptr);
                    
                    //this server doesn't the use the request type properly
                    if ( !ctreq.get_pos(&x, &y, &z, &rx, &ry, &rz, &t) )
                    {
                        ROS_ERROR("couldn't parse cart trajectory point");
                    }
                    else
                    {
                        using namespace descartes_core;
                        using namespace descartes_trajectory;
                        //add a dummy point if this is the first point
                        if (points.empty())
                        {
                            //dummy point
                            std::vector<double> joints;
                            for (int q = 0; q < JOINT_MAX; q++) joints.push_back(0);
                            TrajectoryPtPtr dummy_pt = TrajectoryPtPtr(new JointTrajectoryPt(joints));
                            points.push_back(dummy_pt);
                        }
                        TrajectoryPtPtr next_point = TrajectoryPtPtr(new AxialSymmetricPt(x, y, z, rx, ry, rz, M_PI/2.0, AxialSymmetricPt::Z_AXIS));
                        points.push_back(next_point);
                        times.push_back(t);
                    }
                    ctrep.set_seq_number(ctreq.get_seq_number());
                    ctrep.set_cart_traj_pt_reply(REPLY_SUCCESS);
                    nchars = ulapi_socket_write(args->id, reinterpret_cast<char *>(&ctrep), sizeof(ctrep));
                    if (nchars < 0) {
                        ROS_INFO("Client has left");
                        done = true;
                        break;
                    }
                    break;

                case MESSAGE_JOINT_TRAJ_PT:
                    using namespace descartes_core;
                    using namespace descartes_trajectory;
                    {
                        TrajectoryPtPtr pt;
                        jtreq.read_joint_traj_pt_request(ptr);
                        jtreq.print_joint_traj_pt_request();
                        joints.clear();
                        for (int i = 0; i < JOINT_MAX; i++)
                        {
                            if (! jtreq.get_pos(&x, i)) break;
                            joints.push_back(x);
                        }
                        for(int j = 0; j < joints.size(); j++) ROS_WARN("%d: %f", j, joints[j]);
                        t = 1;
                        pt = TrajectoryPtPtr( new JointTrajectoryPt(joints) );
                        points.push_back(pt);
                        times.push_back(t); 
                    }
                    jtrep.set_joint_traj_pt_reply(REPLY_SUCCESS);
                    nchars = ulapi_socket_write(args->id, reinterpret_cast<char *>(&jtrep), sizeof(jtrep));
                    jtrep.print_joint_traj_pt_reply();
                    if (nchars < 0) done = true;
                    break;

                case MESSAGE_TOGGLE_BATCH:
                    execute = !execute;
                    if (execute)
                    {
                        ROS_INFO("Batching toggled off");
                    }
                    else
                    {
                        ROS_INFO("Batching toggled on");
                    }
                    break;

                default:
                    ROS_WARN("Unknown message type %d", message_type);
            } // switch (message_type)

            if (execute && points.size() > 1)
            {
                times.push_back(1.0);
                if (args->planner.planPath(points) &&  args->planner.getPath(result))
                {
                    trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *(args->model), args->j_names, times);
                    successful_execution = executeTrajectory(joint_solution);
                }
                else
                {
                    ROS_ERROR("Could not generate plan!");
                }
                //set point vector to only contain the last point in the trajectory
                points.clear();
                times.clear();
                if (successful_execution)
                {
                    points.push_back(result.back());
                }
                result.clear();
            }
            //move to next message (if multiple)
            nleft -= (sizeof(length) + length);
            ptr += (sizeof(length) + length);
        } // while (nleft > 0)
    } // while (!done)

    ulapi_socket_close(args->id);
    free(args->thread);
    free(args);
}


/* request server code */
/* spawns a connection request thread for every new connection */
struct request_server_thread_args
{
    int id;
};

static void request_server_thread_code(request_server_thread_args *args)
{
    int connection_id;
    ulapi_task_struct *process_request_thread;
    process_request_thread_args *process_request_args; 
    while (true)
    {
        connection_id = ulapi_socket_get_connection_id(args->id);
        if (connection_id < 0) 
        {
            ROS_ERROR("can't get connection");
            break;
        }
        ROS_INFO("connected on id %d", connection_id);

    // spawn a connection thread
        process_request_thread = reinterpret_cast<ulapi_task_struct *>(malloc(sizeof(*process_request_thread)));
        process_request_args = reinterpret_cast<process_request_thread_args *>(malloc(sizeof(*process_request_args)));

        ulapi_task_init(process_request_thread);
        process_request_args->id = connection_id;
        process_request_args->thread = process_request_thread;
        ulapi_task_start(process_request_thread, reinterpret_cast<ulapi_task_code>(process_request_thread_code), reinterpret_cast<void *>(process_request_args), ulapi_prio_highest(), 0);

    } //while (true)

}

descartes_core::RobotModelPtr process_request_thread_args::model;
descartes_planner::SparsePlanner process_request_thread_args::planner;
std::vector<std::string> process_request_thread_args::j_names;

int main( int argc, char** argv)
{
    /* Descartes Model Initialization */

    const std::string robot_description = argv[1];
    const std::string group_name = argv[2];
    const std::string world_frame = argv[3];
    const std::string tcp_frame = argv[4];

    ros::init(argc, argv, "sm_server_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
        ROS_INFO("Could not initialize robot model");
        exit(-1);
    }
    model->setCheckCollisions(true);
    ROS_INFO("Robot initialized successfully!");
    //descartes_planner::DensePlanner planner;
    descartes_planner::SparsePlanner planner;
    planner.initialize(model);

    // Get Joint Names
    std::vector<std::string> j_names;
    nh.getParam("controller_joint_names", j_names);

    process_request_thread_args::model = model;
    process_request_thread_args::planner = planner;
    process_request_thread_args::j_names = j_names;


    int message_port = MESSAGE_PORT_DEFAULT;
    bool done = false;

    ulapi_task_struct request_server_thread;
    struct request_server_thread_args request_server_args;

    request_server_args.id = ulapi_socket_get_server_id(message_port);
    if (request_server_args.id < 0)
    {
        ROS_ERROR("Unable to serve port %d", message_port);
    }
    else
    {
        ROS_INFO("Started server with id %d", request_server_args.id);
    }

    ulapi_task_init(&request_server_thread);
    ulapi_task_start(   &request_server_thread,
                        reinterpret_cast<ulapi_task_code>(request_server_thread_code),
                        reinterpret_cast<void *>(&request_server_args),
                        ulapi_prio_highest(), 0);

    ROS_INFO("Server waiting for connections, type q to quit");
    while (!done)
    {
        if (fgetc(stdin) == 'q') done = true;
    }
    return 0;
}







