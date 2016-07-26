
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Ostfalia University of Applied Sciences
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Arne Hitzmann */

#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  //Quaternion from torso to desired gripper grasping pose
  tf::Quaternion quat(1,0,0,0);
  quat.setRPY(-M_PI, 0, -M_PI);

  //Pose of grasping destination
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "torso";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.1;
  p.pose.position.z = 0.0;
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  //Grasping Approach for grasping is driving alongside of torso's z-axis
  g.pre_grasp_approach.direction.header.frame_id = "torso";
  g.pre_grasp_approach.direction.vector.z = -1.0;
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.2;

  //Grasping Retreat for grasping is driving alongside of torso's z-axis
  g.post_grasp_retreat.direction.header.frame_id = "torso";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.2;



  //This can be used to send additional joint command e.g gripper positions to make sure it is opened before grasp
  g.pre_grasp_posture.joint_names.push_back("l_gripper_l_finger_joint");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 100;

  //This can be used to send additional joint command e.g gripper positions to make sure it is closed after grasp
  g.grasp_posture.joint_names.push_back("l_gripper_l_finger_joint");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;


  //Pushing this grasp into a vector of possible grasps. In this case we just use one
  grasps.push_back(g);
  group.setSupportSurfaceName("table");

  //Grasp world object part by utilizing grasps. In this step part will get attached to the gripper
  group.pick("part", grasps);
}

void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  //The same as for grasping but no different orientation has to be computed.
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "torso";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.55;
  p.pose.position.z = 0.0;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  //Approach place location from above
  g.pre_place_approach.direction.header.frame_id = "torso";
  g.pre_place_approach.direction.vector.z = -1.0;
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;

  //The same as it was done while picking
  g.post_place_retreat.direction.header.frame_id = "torso";
  g.post_place_retreat.direction.vector.z = 1.0;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.2;


  //In the case of placing we just have one additional command. This was used in the example to open the gripper.
  //This makes sense because while placing we can assume that the gripper is closed anyway to hold the object
  g.post_place_posture.joint_names.push_back("l_gripper_l_finger_joint");
  g.post_place_posture.points.resize(1);
  g.post_place_posture.points[0].positions.resize(1);
  g.post_place_posture.points[0].positions[0] = 100;

  loc.push_back(g);
  group.setSupportSurfaceName("table");

  //This drives to the destination pose and also detaches the object from the gripper
  group.place("part", loc);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();

  //Using the move_group of the left arm
  moveit::planning_interface::MoveGroup group("left_arm");

  /*
  // Constraints to used ad the moment
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "left_endpoint";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  //group.setPathConstraints(constr);*/

  //Set the default trajectory planner to the Rapidly-exploring random tree planner. This is fair enough and fast.
  //group.setPlannerId("RRTConnectkConfigDefault");
  //Set the default trajectory planner to the Expansive Space Trees tree planner. Seems to generate less "fancy" trajectories.
  group.setPlannerId("ESTkConfigDefault");
  group.setPlanningTime(45.0);


  //Adding the objects to the scene. In this example just the "part" to pick is added as a world object which get
  //attached to the gripper later by the pick method and
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "torso";
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitive_poses.resize(1);
  co.primitive_poses[0].orientation.w = 1.0;

  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.8;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = 0.32;
  co.primitive_poses[0].position.z = 0.15;
  pub_co.publish(co);



  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.8;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.8;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = 0.2;
  co.primitive_poses[0].position.z = -0.19;
  pub_co.publish(co);



  //Finally adding the part. First as a world object plus removing any left over attached instances of itself
  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  moveit_msgs::AttachedCollisionObject aco;
  aco.object = co;
  pub_aco.publish(aco);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.04;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.04;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;

  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = 0.1;
  co.primitive_poses[0].position.z = 0.0;
  pub_co.publish(co);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();
  return 0;
}
