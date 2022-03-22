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
   Desc: Provide macros for MoveIt features
*/

#pragma once

#include <moveit/version.h>

/// Create the 'MOVEIT_VERSION_ID' macro which is a numeric identifier.
/// Before 1.1.6, the macro 'MOVEIT_VERSION' was a string. It is now a
/// numeric identifier, thus cannot be used for backward compatibility.
/// https://github.com/ros-planning/moveit/pull/2997

#if !defined(MOVEIT_VERSION_STR) || !defined(MOVEIT_VERSION_CHECK)
/// old code
/// MOVEIT_VERSION is (major << 16) + (minor << 8) + patch.
#define MOVEIT_VERSION_ID MOVEIT_VERSION_CHECK(MOVEIT_VERSION_MAJOR, MOVEIT_VERSION_MINOR, MOVEIT_VERSION_PATCH)

/// Use like: #if MOVEIT_VERSION >= MOVEIT_VERSION_CHECK(1, 0, 0)
#define MOVEIT_VERSION_CHECK(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#else
/// modern code
#define MOVEIT_VERSION_ID MOVEIT_VERSION
#endif

// use object shape poses relative to a single object pose
#define MOVEIT_HAS_OBJECT_POSE MOVEIT_VERSION_ID >= MOVEIT_VERSION_CHECK(1, 1, 6)
