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
   Desc: Update a MoveGroup PlanningScene given a URDF
*/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <urdf_to_scene/scene_parser.h>

void waitForStaticTransform(const SceneParser& parser);

int main(int argc, char** argv) {
    ros::init(argc, argv, "urdf_to_scene");
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
    parser.loadURDF(nh, "/scene_urdf");
    parser.parseURDF();

    // Get conveted planning scene and mark as diff
    auto scene = parser.getPlanningScene();
    scene.is_diff = true;

    // Only needed for the 'load_robot.launch' example
    // Gives time to the move_group PlanningSceneMonitor to find the transform
    if (scene.fixed_frame_transforms.empty())
        waitForStaticTransform(parser);

    // Update the planning scene
    if (!psi.applyPlanningScene(scene))
        ROS_ERROR("Failed to apply the planning scene");

    // Update the collision objects
    // if (!psi.applyCollisionObjects(scene.world.collision_objects))
    //     ROS_ERROR("Failed to apply collision objects");

    return 0;
}

void waitForStaticTransform(const SceneParser& parser) {
    const auto& model = parser.getURDFModel();
    auto root_link = model.getRoot()->name;

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    try {
        tf_buffer->lookupTransform("world", root_link, ros::Time(0), ros::Duration{ 1.0 });
    } catch (tf2::TransformException& ex) {
        // Do nothing
    }
}
