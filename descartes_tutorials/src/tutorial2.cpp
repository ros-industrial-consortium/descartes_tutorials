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

// For reading the position of the grinder from TF
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// For loading the pose file from a local package
#include <ros/package.h>
#include <fstream>
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

  // I won't be repeating too many comments, so please take a look back at the first tutorial for an explanation of
  // the basics.
  ros::AsyncSpinner spinner (1);
  spinner.start();

  // This package assumes that the move group you are using is pointing to an IKFast kinematics plugin in its
  // kinematics.yaml file. By default, it assumes that the underlying kinematics are from 'base_link' to 'tool0'.
  // We're using a group that does not begin or end at these links, but they do exist in the model.
  // If you have renamed these, please set the 'ikfast_base_frame' and 'ikfast_tool_frame' parameter (not in the
  // private namespace) to the base and tool frame used to generate the IKFast model.
  descartes_core::RobotModelPtr model (new descartes_moveit::IkFastMoveitStateAdapter());

  const std::string robot_description = "robot_description";

  // We have made a special move group which goes from the base link of the robot to the reference frame of the mounted
  // part that the robot is holding.
  const std::string group_name = "puzzle";

  // Name of frame in which you are expressing poses.
  const std::string world_frame = "world";

  // In this demo, "part" is a reference frame located on the part that the robot is holding. It's a frame that we can
  // precisely locate w.r.t the robot, and in which we can export the poses representing the edge of the part.
  const std::string tcp_frame = "part";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  model->setCheckCollisions(true); // Let's turn on collision checking.

  // Here we load a path by reading a file of poses generated on the perimeter of the part being held by the robot.
  // These were generated from the CAD model. This is the interesting bit of this tutorial.
  std::vector<descartes_core::TrajectoryPtPtr> points = makePath();

  descartes_planner::DensePlanner planner;

  if (!planner.initialize(model))
  {
    ROS_ERROR("Failed to initialize planner");
    return -2;
  }

  auto start_tm = ros::WallTime::now();
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -3;
  }

  auto search_start_tm = ros::WallTime::now();
  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -4;
  }

  auto end_tm = ros::WallTime::now();

  ROS_INFO_STREAM("Graph building (s): " << (search_start_tm - start_tm).toSec());
  ROS_INFO_STREAM("Graph search (s): " << (end_tm - search_start_tm).toSec());
  ROS_INFO_STREAM("Total time (s): " << (end_tm - start_tm).toSec());

  // Translate the result into something that you can execute.
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

/**
 * @brief Loads a specific file containing the perimeter of the puzzle part generated from CAD. The frames
 * are exported in the "puzzle" frame, so we can coordinate the motion of the robot with this part.
 */
static EigenSTL::vector_Isometry3d makePuzzleToolPoses()
{
  EigenSTL::vector_Isometry3d path; // results
  std::ifstream indata; // input file

  // You could load your parts from anywhere, but we are transporting them with the git repo
  std::string filename = ros::package::getPath("tutorial2_support") + "/config/puzzle_bent.csv";

  // In a non-trivial app, you'll of course want to check that calls like 'open' succeeded
  indata.open(filename);

  std::string line;
  int lnum = 0;
  while (std::getline(indata, line))
  {
      ++lnum;
      if (lnum < 3)
        continue;

      std::stringstream lineStream(line);
      std::string  cell;
      Eigen::Matrix<double, 6, 1> xyzijk;
      int i = -2;
      while (std::getline(lineStream, cell, ','))
      {
        ++i;
        if (i == -1)
          continue;

        xyzijk(i) = std::stod(cell);
      }

      Eigen::Vector3d pos = xyzijk.head<3>();
      pos = pos / 1000.0; // Most things in ROS use meters as the unit of length. Our part was exported in mm.
      Eigen::Vector3d norm = xyzijk.tail<3>();
      norm.normalize();

      // This code computes two extra directions to turn the normal direction into a full defined frame. Descartes
      // will search around this frame for extra poses, so the exact values do not matter as long they are valid.
      Eigen::Vector3d temp_x = (-1 * pos).normalized();
      Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
      Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
      Eigen::Isometry3d pose;
      pose.matrix().col(0).head<3>() = x_axis;
      pose.matrix().col(1).head<3>() = y_axis;
      pose.matrix().col(2).head<3>() = norm;
      pose.matrix().col(3).head<3>() = pos;

      path.push_back(pose);
  }
  indata.close();

  return path;
}

std::vector<descartes_core::TrajectoryPtPtr>
makeDescartesTrajectory(const EigenSTL::vector_Isometry3d& path)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  std::vector<descartes_core::TrajectoryPtPtr> descartes_path; // return value

  // This is where the magic happens. We have a tool path, but its in a coordinate frame that is local
  // to the part. We'll use a special constructor of CartTrajectoryPt to tell Descartes about this soon,
  // but first we need to figure out where the grinder is in our "world". You might have this hard coded,
  // but here we're going to lookup its position. Note that we are looking up the frame of the static tool
  // in the base frame of the Descartes model given in the initialize() function in main().
  tf::TransformListener listener;
  tf::StampedTransform grinder_frame;
  listener.waitForTransform("world", "grinder_frame", ros::Time(0), ros::Duration(5.0));
  listener.lookupTransform("world", "grinder_frame", ros::Time(0), grinder_frame);

  // Descartes uses eigen, so let's convert the data type
  Eigen::Isometry3d gf;
  tf::transformTFToEigen(grinder_frame, gf);

  // When you tell Descartes about a "cartesian" frame, you can specify 4 "supporting" frames, and all are
  // identity unless otherwise stated:
  // 1. 'wobj_base': A fixed frame from the base of the robot to the base of a part.
  // 2. 'wobj_pt': The point at which the 'tool_pt' should be put, specified in the frame of 'wobj_base'.
  //               This point can have tolerances associated with it.
  // 3. 'tool_base': A fixed frame from the tool of the robot (e.g. tool0) to a reference point on the tool.
  // 4. 'tool_pt': The point which is put on 'wobj_pt' specified in the frame of 'tool_base'. This point can
  //               have tolerances associated with it.
  //
  // NOTE: I'm pretty sure the math is broken if you specify tolerances for BOTH 'wobj_pt' and 'tool_pt' at the
  // same time. I recommend just using one toleranced frame.

  Frame wobj_base(gf); // Here we say our wobj_base is the grinder pose
  Frame tool_base = Frame::Identity(); // That our tool base is identity. This is because the URDF frame "puzzle" already
                                       // puts us in the right frame. If we didn't have this, or it changed a lot, we
                                       // could use this value to set the base frame for our CAD points
  TolerancedFrame wobj_pt = Frame::Identity(); // Here we keep the base frame identical to the grinder pose.

  // Now we translate our poses to Descartes points
  for (const auto& point : path)
  {
    // Each 'point' represents a point on the edge of the puzzle part. The tool poses are exported in such a way that
    // the Z of each point is parallel with the grinder, so turning around Z is okay.
    TolerancedFrame tool_pt(point);
    tool_pt.orientation_tolerance.z_lower = -M_PI; // Search -PI to PI (so 360 degrees)
    tool_pt.orientation_tolerance.z_upper = M_PI;

    boost::shared_ptr<CartTrajectoryPt> pt(
          new CartTrajectoryPt(wobj_base, wobj_pt, tool_base, tool_pt, // Here we specify our frames
                               0, // Don't search in cartesian space. We want the points to be exact.
                               M_PI/90.0, // Do search our rotation in 2 degree increments.
                               descartes_core::TimingConstraint(0.25))); // Make every point 0.25 seconds apart
    descartes_path.push_back(pt);
  }
  return descartes_path;
}

std::vector<descartes_core::TrajectoryPtPtr> makePath()
{
  EigenSTL::vector_Isometry3d tool_poses = makePuzzleToolPoses();
  return makeDescartesTrajectory(tool_poses);
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
