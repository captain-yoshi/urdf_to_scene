/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
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
 *   * Neither the name of Bielefeld University nor the names of its
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

/* Author: Captain Yoshi
   Desc:
*/

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <scene_parser/scene_parser.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "scene_parser");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	// Remove all scene objects
	moveit::planning_interface::PlanningSceneInterface psi;
	{
		moveit_msgs::PlanningScene rm;
		rm.is_diff = true;
		rm.robot_state.is_diff = true;
		rm.robot_state.attached_collision_objects.resize(1);
		rm.robot_state.attached_collision_objects[0].object.operation = moveit_msgs::CollisionObject::REMOVE;
		rm.world.collision_objects.resize(1);
		rm.world.collision_objects[0].operation = moveit_msgs::CollisionObject::REMOVE;
		psi.applyPlanningScene(rm);
	}

	// Parse URDF into a planning scene
	SceneParser parser;
	parser.loadURDFFile(nh, "/scene_urdf");
	const auto& ps = parser.getPlanningScene();

	// Add collision objects to the planning scene
	if (!psi.applyCollisionObjects(ps.world.collision_objects)) {
		ROS_ERROR("Failed to apply collision objects");
	}

	/*
	// Debug
	std::string file;
	if (!nh.getParam("/scene_urdf", file))
	   ROS_ERROR("Robot start states not found");

	std::cout << file;
	*/

	return 0;
}
