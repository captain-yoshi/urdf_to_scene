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
 *   * Neither the name of the copyright holder nor the names of its
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
   Desc: Parser for converting a urdf into a MoveIt PlanningScene
*/

#pragma once

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <urdf/model.h>
#include <moveit_msgs/PlanningScene.h>

class SceneParser
{
public:
    bool loadURDF(ros::NodeHandle& nh, const std::string& param_name);
    bool loadURDF(const std::string& urdf_str);

    /// load urdf first
    void parseURDF();

    /// retrieve data
    const moveit_msgs::PlanningScene& getPlanningScene() const;
    const urdf::Model& getURDFModel() const;
    const std::map<std::string, std::string>& getMeshResourceMap() const;

    void printTF(const std::string& tf_name, const Eigen::Isometry3d& tf);

private:
    void parseFixedFrameTransforms();
    void parseLink(const urdf::LinkConstSharedPtr& rchild_link, const Eigen::Isometry3d& offset,
                   std::map<std::string, std::string>& dummy_link_names);
    void parseCollisionGeometry(const urdf::LinkConstSharedPtr& link, const std::string& frame_id,
                                const Eigen::Isometry3d& parent_to_collision_tf);

    void createCollisionObjectPrimitive(moveit_msgs::CollisionObject& collision_object, const std::string& object_id,
                                        const shape_msgs::SolidPrimitive& primitive, const Eigen::Isometry3d& frame,
                                        const std::string& frame_id);
    void createCollisionObjectMesh(moveit_msgs::CollisionObject& collision_object, const std::string& object_id,
                                   const std::string& mesh_path, const Eigen::Isometry3d& frame,
                                   const std::string& frame_id, const Eigen::Vector3d& scaling);

    urdf::Model model_;
    tinyxml2::XMLDocument xml_;
    moveit_msgs::PlanningScene scene_;

    std::map<std::string, std::string> mesh_resource_map_;  // {COLLISION_OBJECT_ID, MESH_FILENAME}
};
