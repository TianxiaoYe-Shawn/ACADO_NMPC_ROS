// ACADO
#include "../acado_mpc_export/acado_common.h"
#include "../acado_mpc_export/acado_auxiliary_functions.h"

// Eigen
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// C++
#include <iostream>
#include <vector>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelStates.h"

using namespace std;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

#define Ts 0.1 // sampling time
#define Lf 1.0

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// MPC init functions
vector<vector<double>> init_acado();
void init_weight();
vector<vector<double>> run_mpc_acado(vector<double> states,
                                    vector<double> ref_states,
                                    vector<vector<double>> previous_u);
vector<double> calculate_ref_states(const vector<double> &ref_x,
									const vector<double> &ref_y,
									const vector<double> &ref_q,
                                    const double &reference_v,
                                    const double &reference_w);
vector<double> motion_prediction(const vector<double> &cur_states,
                                const vector<vector<double>> &prev_u);
vector<double> update_states(vector<double> state, double v_cmd, double w_cmd);


/* ROS PARAMS*/
double weight_x, weight_y, weight_q, weight_v, weight_w;

geometry_msgs::Pose odom_pose;
geometry_msgs::Twist odom_twist;
void stateCallback(const gazebo_msgs::ModelStates& msg) {
    for (size_t i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == "jackal") {
            odom_pose = msg.pose[i];
            odom_twist = msg.twist[i];
            break;
        }
    }
}

bool receive_traj = false;
nav_msgs::Path path;
void pathCallback(const nav_msgs::Path& msg) 
{ 
	if (!receive_traj) 
	{ 
		path = msg; 
		receive_traj = true; 
		ROS_INFO("Received new trajectory with %d poses.", int(msg.poses.size()));
	} 
	else 
	{
        ROS_INFO("New trajectory received but already have one.");
    }
}

bool is_target(geometry_msgs::Pose cur, double goal_x, double goal_y)
{
	if(abs(cur.position.x - goal_x) < 0.05 && abs(cur.position.y - goal_y) < 0.05)
	{
		return true;
	}
	else return false;
}

float quaternion2Yaw(geometry_msgs::Quaternion orientation)
{
    double q0 = orientation.x;
    double q1 = orientation.y;
    double q2 = orientation.z;
    double q3 = orientation.w;

    float yaw = atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2));
    return yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe("/gazebo/model_states", 10, stateCallback);
    ros::Subscriber path_sub = nh.subscribe("/trajectory", 1, pathCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Publisher predict_pub = nh.advertise<nav_msgs::Path>("/predict_path", 10);

	/* ROS PARAM */
	ros::param::get("~weight_x", weight_x);
	ros::param::get("~weight_y", weight_y);
	ros::param::get("~weight_q", weight_q);
	ros::param::get("~weight_v", weight_v);
	ros::param::get("~weight_w", weight_w);

	cout << weight_x << " " << weight_y << " " << weight_q << endl;	

    vector<vector<double>> control_output;
    control_output = init_acado();

    // cout << control_output[0][0] << endl;
	double start_time = ros::Time::now().toSec();
    ros::Rate r(10);

	double goal_x, goal_y;
	int count = 0;
    while(ros::ok())
    {
		if (path.poses.size() == 0)
		{
			ros::spinOnce();
			r.sleep();
			continue;
		}

		goal_x = path.poses[path.poses.size()-1].pose.position.x;
		goal_y = path.poses[path.poses.size()-1].pose.position.y;

		// State
		double px = odom_pose.position.x; // state[0];
		double py = odom_pose.position.y; // state[1];

		double q0 = odom_pose.orientation.x;
		double q1 = odom_pose.orientation.y;
		double q2 = odom_pose.orientation.z;
		double q3 = odom_pose.orientation.w;
		double t0 = -2.0 * (q1*q1 + q2*q2) + 1.0;
		double t1 = +2.0 * (q2*q3 + q0*q1);

		double pq = atan2(t1, t0); // state[2]; 

		vector<double> cur_state = {px, py, pq};

		// Reference state
		vector<double> ptsx, ptsy, ptsq;

		for (int i = 0; i < ACADO_N; i++)
		{			
			double pred_x, pred_y, pred_q;
			if (count + i >= path.poses.size())
			{
				pred_x = path.poses[path.poses.size()-1].pose.position.x;
				pred_y = path.poses[path.poses.size()-1].pose.position.y;
				pred_q = quaternion2Yaw(path.poses[path.poses.size()-1].pose.orientation);
				ptsx.push_back(pred_x);
				ptsy.push_back(pred_y);
				ptsq.push_back(pred_q);
			}
			else
			{
				pred_x = path.poses[count+i].pose.position.x;
				pred_y = path.poses[count+i].pose.position.y;
				pred_q = quaternion2Yaw(path.poses[count+i].pose.orientation);
				ptsx.push_back(pred_x);
				ptsy.push_back(pred_y);
				ptsq.push_back(pred_q);
			}
		}

		double reference_v = 0.0;
		double reference_w = 0.0;

        // ACADO
        vector<double> predicted_states = motion_prediction(cur_state, control_output);
        vector<double> ref_states = calculate_ref_states(ptsx, ptsy, ptsq, reference_v, reference_w);
        control_output = run_mpc_acado(predicted_states, ref_states, control_output);

		// Predict path
		nav_msgs::Path predict_path;
		predict_path.header.frame_id = "/map";
		predict_path.header.stamp = ros::Time::now();
		for (int i = 0; i < ACADO_N; i++)
		{
			geometry_msgs::PoseStamped pred_pose;
			pred_pose.header = predict_path.header;
			pred_pose.pose.position.x = acadoVariables.x[NX * i + 0];
			pred_pose.pose.position.y = acadoVariables.x[NX * i + 1];
			pred_pose.pose.orientation.w = 1.0;
			predict_path.poses.push_back(pred_pose);
		}
		predict_pub.publish(predict_path);
		// cout << "Oke" << endl;

		geometry_msgs::Twist vel;

		// Check target
		bool goal = is_target(odom_pose, goal_x, goal_y) && count >= path.poses.size();
		if(goal)
		{
			vel.linear.x = 0;
			vel.angular.z = 0;
		}
		else
		{
			vel.linear.x = control_output[0][0];
			vel.angular.z = control_output[1][0];
		}
		vel_pub.publish(vel);

        ros::spinOnce();
        r.sleep();
		count++;
    }
    return 0;
}

void init_weight()
{
	for (int i = 0; i < N; i++)
	{
		// Setup diagonal entries
		acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weight_x;
		acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weight_y;
		acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weight_q;
		acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weight_v;
		acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weight_w;
	}
	acadoVariables.WN[(NYN + 1) * 0] = weight_x;
	acadoVariables.WN[(NYN + 1) * 1] = weight_y;
	acadoVariables.WN[(NYN + 1) * 2] = weight_q;
}

vector<vector<double>> init_acado()
{
    /* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (int i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (int i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (int i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (int i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

    acado_preparationStep();
	vector<double> control_output_v;
    vector<double> control_output_w;
	for (int i = 0; i < ACADO_N; ++i)
	{
        // There are 2 outputs v, w
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_v.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
			else 
			{
				control_output_w.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
		}
	}
	init_weight();
	return {control_output_v, control_output_w};
}

vector<vector<double>> run_mpc_acado(vector<double> states,
                                    vector<double> ref_states,
                                    vector<vector<double>> previous_u)
{
    /* Some temporary variables. */
	int i, iter;
	acado_timer t;

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  
	{
		acadoVariables.x[ i ] = (real_t) states[i];
	}
    for (i = 0; i < NX; ++i)
	{
		acadoVariables.x0[i] = (real_t)states[i];
	}

    /* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)
	{
		acadoVariables.y[i] = (real_t)ref_states[i];
	}
	for (i = 0; i < NYN; ++i)
	{
		acadoVariables.yN[i] = (real_t)ref_states[NY * (N - 1) + i];
	}

	// /* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic(&t);

	/* The "real-time iterations" loop. */

	for (iter = 0; i < NUM_STEPS; ++i)
	{
		/* Perform the feedback step. */
		acado_feedbackStep();
		acado_preparationStep();

	}

	/* Read the elapsed time. */
	real_t te = acado_toc(&t);

    vector<double> control_output_v;
    vector<double> control_output_w;
	real_t *u = acado_getVariablesU();
	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_v.push_back((double)u[i * ACADO_NU + j]);
			}
			else
			{
				control_output_w.push_back((double)u[i * ACADO_NU + j]);
			}
		}
	}
	// cout << control_output_v[0] << " " << control_output_w[0] << endl;
	return {control_output_v, control_output_w};
}

vector<double> calculate_ref_states(const vector<double> &ref_x,
									const vector<double> &ref_y,
									const vector<double> &ref_q,
                                    const double &reference_v,
                                    const double &reference_w)
{
	vector<double> result;
	for (int i = 0; i < N; i++)
	{
		result.push_back(ref_x[i]);
		result.push_back(ref_y[i]);
		result.push_back(ref_q[i]);
		result.push_back(0);
		result.push_back(0);
	}
	return result;
}

vector<double> update_states(vector<double> state, double v_cmd, double w_cmd)
{
	// based on kinematic model
	double x0 = state[0];
	double y0 = state[1];
	double q0 = state[2];
	double v0 = v_cmd;
	double w0 = w_cmd;

	double x1 = x0 + (v0 * cos(q0))* Ts;
	double y1 = y0 + (v0 * sin(q0))* Ts;
	double q1 = q0 + w0 * Ts;
	return {x1, y1, q1};
}

vector<double> motion_prediction(const vector<double> &cur_states,
                                const vector<vector<double>> &prev_u)
{
	vector<double> old_v_cmd = prev_u[0];
	vector<double> old_w_cmd = prev_u[1];

	vector<vector<double>> predicted_states;
	predicted_states.push_back(cur_states);

	for (int i = 0; i < N; i++)
	{
		vector<double> cur_state = predicted_states[i];
		// yaw angle compensation of overflow
		if (cur_state[3] > M_PI)
		{
			cur_state[3] -= 2 * M_PI;
		}
		if (cur_state[3] < -M_PI)
		{
			cur_state[3] += 2 * M_PI;
		}
		vector<double> next_state = update_states(cur_state, old_v_cmd[i], old_w_cmd[i]);
		predicted_states.push_back(next_state);
	}

	vector<double> result;
	for (int i = 0; i < (ACADO_N + 1); ++i)
	{
		for (int j = 0; j < NX; ++j)
		{
			result.push_back(predicted_states[i][j]);
		}
	}
	return result;
}

