// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/ikfast_moveit_state_adapter.h>

// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

// Includes the utility function for converting to trajectory_msgs::JointTrajectory's
#include <descartes_utilities/ros_conversions.h>

/**
 * Makes a dummy trajectory for the robot to follow.
 */
std::vector<descartes_core::TrajectoryPtPtr> makePath();

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  // Since we're not calling ros::spin() and doing the planning in a callback, but rather just handling this
  // inline, we need to create an async spinner if our publishers are to work. Note that many MoveIt components
  // will also not work without an active spinner and Descartes uses moveit for its "groups" and "scene" descriptions
  ros::AsyncSpinner spinner (1);
  spinner.start();

  // 1. First thing first, let's create a kinematic model of the robot. In Descartes, this is used to do things
  // like forward kinematics (joints -> pose), inverse kinematics (pose -> many joints), and collision checking.

  // All of the existing planners (as of Nov 2017) have been designed with the idea that you have "closed form"
  // kinematics. This means that the default solvers in MoveIt (KDL) will NOT WORK WELL. I encourage you to produce
  // an ikfast model for your robot (see MoveIt tutorial) or use the OPW kinematics package if you have a spherical
  // wrist industrial robot. See http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html 

  // This package assumes that the move group you are using is pointing to an IKFast kinematics plugin in its
  // kinematics.yaml file. By default, it assumes that the underlying kinematics are from 'base_link' to 'tool0'.
  // If you have renamed these, please set the 'ikfast_base_frame' and 'ikfast_tool_frame' parameter (not in the
  // private namespace) to the base and tool frame used to generate the IKFast model.
  descartes_core::RobotModelPtr model (new descartes_moveit::IkFastMoveitStateAdapter());

  // Name of description on parameter server. Typically just "robot_description". Used to initialize
  // moveit model.
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant. For many industrial robots this will be
  // "manipulator"
  const std::string group_name = "manipulator";

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = "base_link";

  // tool center point frame (name of link associated with tool). The robot's flange is typically "tool0" but yours
  // could be anything. We typically have our tool's positive Z-axis point outward from the grinder, welder, etc.
  const std::string tcp_frame = "tool0";

  // Before you can use a model, you must call initialize. This will load robot models and sanity check the model.
  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  model->setCheckCollisions(true); // Let's turn on collision checking.

  // 2. The next thing to do is to generate a path for the robot to follow. The description of this path is one of the
  // cool things about Descartes. The source of this path is where this library ties into your application: it could
  // come from CAD or from surfaces that were "scanned".

  // Make the path by calling a helper function. See makePath()'s definition for more discussion about paths.
  std::vector<descartes_core::TrajectoryPtPtr> points = makePath();

  // 3. Now we create a planner that can fuse your kinematic world with the points you want to move the robot
  // along. There are a couple of planners now. DensePlanner is the naive, brute force approach to solving the
  // trajectory. SparsePlanner may be faster for some problems (especially very dense ones), but has recieved
  // less overall testing and evaluation.
  descartes_planner::DensePlanner planner;

  // Like the model, you also need to call initialize on the planner
  if (!planner.initialize(model))
  {
    ROS_ERROR("Failed to initialize planner");
    return -2;
  }

  // 4. Now, for the planning itself. This typically happens in two steps. First, call planPath(). This function takes
  // your input trajectory and expands it into a large kinematic "graph". Failures at this point indicate that the
  // input path may not have solutions at a given point (because of reach/collision) or has two points with no way
  // to connect them.
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -3;
  }

  // After expanding the graph, we now call 'getPath()' which searches the graph for a minimum cost path and returns
  // the result. Failures here (assuming planPath was good) indicate that your path has solutions at every waypoint
  // but constraints prevent a solution through the whole path. Usually this means a singularity is hanging out in the
  // middle of your path: the robot can solve all the points but not in the same arm configuration.
  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -4;
  }

  // 5. Translate the result into something that you can execute. In ROS land, this means that we turn the result into
  // a trajectory_msgs::JointTrajectory that's executed through a control_msgs::FollowJointTrajectoryAction. If you
  // have your own execution interface, you can get joint values out of the results in the same way.

  // get joint names - this could be from the robot model, or from the parameter server.
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);

  // Create a JointTrajectory
  trajectory_msgs::JointTrajectory joint_solution;
  joint_solution.joint_names = names;

  // Define a default velocity. Descartes points without specified timing will use this value to limit the
  // fastest moving joint. This usually effects the first point in your path the most.
  const static double default_joint_vel = 0.5; // rad/s
  if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
  {
    ROS_ERROR("Unable to convert Descartes trajectory to joint points");
    return -5;
  }

  // 6. Send the ROS trajectory to the robot for execution
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -6;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}

std::vector<descartes_core::TrajectoryPtPtr> makePath()
{
  // In Descartes, trajectories are composed of "points". Each point describes what joint positions of the robot can
  // satisfy it. You can have a "joint point" for which only a single solution is acceptable. You might have a
  // fully defined cartesian point for which many (8 or 16) different robot configurations might work. You could
  // allow extra tolerances in any of these and even more points satisfy the constraints.

  // In this first tutorial, we're just going to describe a simple cartesian trajectory that moves the robot
  // along a line in the XY plane.

  // Step 1: Let's start by just doing the math to generate the poses we want.

  // First thing, let's generate a pattern with its origin at zero. We'll define another transform later that
  // can move it to somewere more convenient.
  const static double step_size = 0.01;
  const static int num_steps = 20;
  const static double time_between_points = 0.5;

  EigenSTL::vector_Isometry3d pattern_poses;
  for (int i = -num_steps / 2; i < num_steps / 2; ++i)
  {
    // Create a pose and initialize it to identity
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    // set the translation (we're moving along a line in Y)
    pose.translation() = Eigen::Vector3d(0, i * step_size, 0);
    // set the orientation. By default, the tool will be pointing up into the air when we usually want it to
    // be pointing down into the ground.
    pose *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // this flips the tool around so that Z is down
    pattern_poses.push_back(pose);
  }

  // Now lets translate these points to Descartes trajectory points
  // The ABB2400 is pretty big, so let's move the path forward and up.
  Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
  pattern_origin.translation() = Eigen::Vector3d(1.0, 0, 0.3);

  std::vector<descartes_core::TrajectoryPtPtr> result;
  for (const auto& pose : pattern_poses)
  {
    // This creates a trajectory that searches around the tool Z and let's the robot move in that null space
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_between_points);
    // This creates a trajectory that is rigid. The tool cannot float and must be at exactly this point.
    //  descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * pose, time_between_points);
    result.push_back(pt);
  }

  // Note that we could also add a joint point representing the starting location of the robot, or a joint point
  // representing the desired end pose of the robot to the front and back of the vector respectively.

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  
  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}
