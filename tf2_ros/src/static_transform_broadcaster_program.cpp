/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster_node.hpp>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

static std::string parse_args(
  const std::vector<std::string> & args,
  bool & help,
  tf2::Quaternion & quat,
  tf2::Vector3 & trans,
  std::string & frame_id,
  std::string & child_frame_id)
{
  size_t size = args.size();
  size_t last_index = size - 1;

  bool saw_quat_flag = false;
  bool saw_rpy_flag = false;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  bool saw_trans_flag = false;

  bool saw_frame_flag = false;

  std::vector<std::string> no_flag_args;

  if (size < 1) {
    return "Not enough arguments to parse";
  }

  help = false;

  size_t i = 1;
  while (i < size) {
    if (args[i] == "-h" || args[i] == "--help") {
      help = true;
      return "";
    } else if (args[i] == "--qx") {
      if (i == last_index) {
        return "Not enough arguments for --qx";
      }

      saw_quat_flag = true;
      try {
        quat.setX(std::stod(args[++i]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --qx argument as float";
      }
    } else if (args[i] == "--qy") {
      if (i == last_index) {
        return "Not enough arguments for --qy";
      }

      saw_quat_flag = true;
      try {
        quat.setY(std::stod(args[++i]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --qy argument as float";
      }
    } else if (args[i] == "--qz") {
      if (i == last_index) {
        return "Not enough arguments for --qz";
      }

      saw_quat_flag = true;
      try {
        quat.setZ(std::stod(args[++i]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --qz argument as float";
      }
    } else if (args[i] == "--qw") {
      if (i == last_index) {
        return "Not enough arguments for --qw";
      }

      saw_quat_flag = true;
      try {
        quat.setW(std::stod(args[++i]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --qw argument as float";
      }
    } else if (args[i] == "--roll") {
      if (i == last_index) {
        return "Not enough arguments for --roll";
      }

      saw_rpy_flag = true;
      try {
        roll = std::stod(args[++i]);
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --roll argument as float";
      }
    } else if (args[i] == "--pitch") {
      if (i == last_index) {
        return "Not enough arguments for --pitch";
      }

      saw_rpy_flag = true;
      try {
        pitch = std::stod(args[++i]);
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --pitch argument as float";
      }
    } else if (args[i] == "--yaw") {
      if (i == last_index) {
        return "Not enough arguments for --yaw";
      }

      saw_rpy_flag = true;
      try {
        yaw = std::stod(args[++i]);
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --yaw argument as float";
      }
    } else if (args[i] == "--x") {
      if (i == last_index) {
        return "Not enough arguments for --x";
      }

      saw_trans_flag = true;
      try {
        trans.setX(std::stod(args[++i]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --x argument as float";
      }
    } else if (args[i] == "--y") {
      if (i == last_index) {
        return "Not enough arguments for --y";
      }

      saw_trans_flag = true;
      try {
        trans.setY(std::stod(args[++i]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --y argument as float";
      }
    } else if (args[i] == "--z") {
      if (i == last_index) {
        return "Not enough arguments for --z";
      }

      saw_trans_flag = true;
      try {
        trans.setZ(std::stod(args[++i]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse --z argument as float";
      }
    } else if (args[i] == "--frame-id") {
      if (i == last_index) {
        return "Not enough arguments for --frame-id";
      }

      saw_frame_flag = true;
      frame_id = args[++i];
    } else if (args[i] == "--child-frame-id") {
      if (i == last_index) {
        return "Not enough arguments for --child-frame-id";
      }

      saw_frame_flag = true;
      child_frame_id = args[++i];
    } else {
      no_flag_args.push_back(args[i]);
    }

    ++i;
  }

  if (saw_rpy_flag && saw_quat_flag) {
    return "Cannot specify both quaternion and Euler rotations";
  } else if (saw_rpy_flag) {
    quat.setRPY(roll, pitch, yaw);
  }

  if (no_flag_args.size() == 8 || no_flag_args.size() == 9) {
    RCUTILS_LOG_WARN("Old-style arguments are deprecated; see --help for new-style arguments");
    if (saw_frame_flag || saw_trans_flag || saw_quat_flag) {
      return "Cannot specify both new-style (flags) and old-style (arguments)";
    }
    try {
      trans.setX(std::stod(no_flag_args[0]));
    } catch (const std::invalid_argument & e) {
      return "Failed to parse X argument as float";
    }
    try {
      trans.setY(std::stod(no_flag_args[1]));
    } catch (const std::invalid_argument & e) {
      return "Failed to parse Y argument as float";
    }
    try {
      trans.setZ(std::stod(no_flag_args[2]));
    } catch (const std::invalid_argument & e) {
      return "Failed to parse Z argument as float";
    }

    if (no_flag_args.size() == 8) {
      try {
        yaw = std::stod(no_flag_args[3]);
      } catch (const std::invalid_argument & e) {
        return "Failed to parse yaw argument as float";
      }
      try {
        pitch = std::stod(no_flag_args[4]);
      } catch (const std::invalid_argument & e) {
        return "Failed to parse pitch argument as float";
      }
      try {
        roll = std::stod(no_flag_args[5]);
      } catch (const std::invalid_argument & e) {
        return "Failed to parse roll argument as float";
      }

      quat.setRPY(roll, pitch, yaw);
      frame_id = no_flag_args[6];
      child_frame_id = no_flag_args[7];
    } else {
      try {
        quat.setX(std::stod(no_flag_args[3]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse qx argument as float";
      }
      try {
        quat.setY(std::stod(no_flag_args[4]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse qy argument as float";
      }
      try {
        quat.setZ(std::stod(no_flag_args[5]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse qz argument as float";
      }
      try {
        quat.setW(std::stod(no_flag_args[6]));
      } catch (const std::invalid_argument & e) {
        return "Failed to parse qw argument as float";
      }
      frame_id = no_flag_args[7];
      child_frame_id = no_flag_args[8];
    }
  } else if (no_flag_args.size() != 0) {
    return "Extra unparsed arguments on command-line";
  }

  if (frame_id == "") {
    return "Frame id must not be empty";
  }

  if (child_frame_id == "") {
    return "Child frame id must not be empty";
  }

  return "";
}

static void _print_usage()
{
  const char * usage =
    "usage: static_transform_publisher [--x X] [--y Y] [--z Z] [--qx QX] [--qy QY] "
    "[--qz QZ] [--qw QW] [--roll ROLL] [--pitch PITCH] [--yaw YAW] --frame-id FRAME_ID "
    "--child-frame-id CHILD_FRAME_ID\n\n"
    "A command line utility for manually sending a static transform.\n\nIf no translation or"
    " orientation is provided, the identity transform will be published.\n\nThe translation offsets"
    " are in meters.\n\nThe rotation may be provided with roll, pitch, yaw euler angles in radians,"
    " or as a quaternion.\n\n"
    "required arguments:\n"
    "  --frame-id FRAME_ID parent frame\n"
    "  --child-frame-id CHILD_FRAME_ID child frame id\n\n"
    "optional arguments:\n"
    "  --x X                 x component of translation\n"
    "  --y Y                 y component of translation\n"
    "  --z Z                 z component of translation\n"
    "  --qx QX               x component of quaternion rotation\n"
    "  --qy QY               y component of quaternion rotation\n"
    "  --qz QZ               z component of quaternion rotation\n"
    "  --qw QW               w component of quaternion rotation\n"
    "  --roll ROLL           roll component Euler rotation\n"
    "  --pitch PITCH         pitch component Euler rotation\n"
    "  --yaw YAW             yaw component Euler rotation";
  printf("%s\n", usage);
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  bool help = false;
  tf2::Quaternion rotation(0.0, 0.0, 0.0, 1.0);
  tf2::Vector3 translation(0.0, 0.0, 0.0);
  std::string frame_id;
  std::string child_frame_id;

  std::string ret = parse_args(args, help, rotation, translation, frame_id, child_frame_id);
  if (ret != "") {
    RCUTILS_LOG_ERROR("error parsing command line arguments: %s", ret.c_str());
    _print_usage();
    return 1;
  }
  if (help) {
    _print_usage();
    return 0;
  }

  rclcpp::NodeOptions options;
  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"translation.x", translation.x()},
    {"translation.y", translation.y()},
    {"translation.z", translation.z()},
    {"rotation.x", rotation.x()},
    {"rotation.y", rotation.y()},
    {"rotation.z", rotation.z()},
    {"rotation.w", rotation.w()},
    {"frame_id", frame_id},
    {"child_frame_id", child_frame_id},
  });

  std::shared_ptr<tf2_ros::StaticTransformBroadcasterNode> node;

  node = std::make_shared<tf2_ros::StaticTransformBroadcasterNode>(options);

  RCLCPP_INFO(
    node->get_logger(),
    "Spinning until killed publishing transform\ntranslation: ('%lf', '%lf', '%lf')\n"
    "rotation: ('%lf', '%lf', '%lf', '%lf')\nfrom '%s' to '%s'",
    translation.x(), translation.y(), translation.z(),
    rotation.x(), rotation.y(), rotation.z(), rotation.w(),
    frame_id.c_str(), child_frame_id.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
