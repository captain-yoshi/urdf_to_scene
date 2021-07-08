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

#pragma once

#include <ros/ros.h>

#include <urdf/model.h>

#include <moveit_msgs/PlanningScene.h>
#include <eigen_conversions/eigen_msg.h>

class SceneParser
{
public:
	bool loadURDFFile(ros::NodeHandle& nh, const std::string& param_name);

	void getCollisionObjects(std::vector<moveit_msgs::CollisionObject>& collision_objects);
	const moveit_msgs::PlanningScene& getPlanningScene();

private:
	void parseURDFmodel();
	void parseChildLink(const urdf::LinkSharedPtr& rchild_link, const Eigen::Isometry3d& offset,
	                    std::map<std::string, std::string>& dummy_link_names);

	void urdfPoseToEigenIsometry(const urdf::Pose& urdf_pose, Eigen::Isometry3d& eigen_pose);

	void createCollisionObjectPrimitive(moveit_msgs::CollisionObject& collision_object, const std::string& object_id,
	                                    const shape_msgs::SolidPrimitive& primitive, const Eigen::Isometry3d& frame,
	                                    const std::string& frame_id);
	void createCollisionObjectMesh(moveit_msgs::CollisionObject& collision_object, const std::string& object_id,
	                               const std::string& mesh_path, const Eigen::Isometry3d& frame,
	                               const std::string& frame_id, const Eigen::Vector3d& scaling);

	std::string urdf_;
	urdf::Model model_;
	moveit_msgs::PlanningScene scene_;
};
