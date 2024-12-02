// C++ Libraries
#include <iostream>
#include <sstream>
#include <string>

// C++ Libraries for handling .yaml files
#include <yaml-cpp/yaml.h>

// ROS Libraries
#include <ros/ros.h>

// ROS Messages used
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

// tf2_ros to broadcast markers poses
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>  // tf2 library for Quaternions
#include <tf2/LinearMath/Transform.h>   // tf2 library for Transforms

#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // to convert between tf2 and geometry_msgs


/*
 * STRUCTS
 */

// landmarks structure
struct Landmark {
    int id;
    geometry_msgs::Pose landmark_pose_;
};

// map structure
struct Map {
    std::string name_;
    std::string frame_id_;
    int num_landmarks_;
    std::vector<Landmark> landmarks_;
};


// FUNCTION DECLARATIONS

/*
 * MAIN FUNCTION
 */
int main(int argc, char** argv)
{
    bool debug_mode = true; // debug mode for the tf2 buffer

    // -------------------------------------------
    // ROS INITIALIZATION AND CONFIGURATIONS
    //
    //
    ros::init(argc, argv, "aruco_localization_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Publisher for robot's pose
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);

    // TF2 Buffer and Listener
    tf2_ros::Buffer tfBuffer(ros::Duration(10), debug_mode);
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster; // Static TF broadcaster to broadcast the map's landmarks

    // get frame_ids to use from the parameters passed in the launcher
    std::string base_frame;
    std::string camera_frame;
    nh_private.param<std::string>("base_frame", base_frame, nh_private.getNamespace() + "/base_link");
    nh_private.param<std::string>("camera_frame", camera_frame, nh_private.getNamespace() + "/camera_link");


    // -------------------------------------
    // LOAD THE MAP FILE
    //
    // load the .YAML map file with the locations of each marker in the world frame
    //
    std::string map_file_path;
    nh_private.param<std::string>("map_file_path", map_file_path, "./src/localization_aruco/maps/");
    std::string map_file_name;
    nh_private.param<std::string>("map_file_name", map_file_name, "map.yaml");

    // Map structure to store the map data
    Map mapData;

    try
    {
        // Load the map file
        YAML::Node map = YAML::LoadFile(map_file_path + map_file_name);

        // Store the map information in a Map structure
        mapData.name_ = map["name"].as<std::string>();
        mapData.frame_id_ = map["frame_id"].as<std::string>();
        mapData.num_landmarks_ = map["num_landmarks"].as<int>();

        // Print the map file information
        ROS_INFO("Map Name: %s", map["name"].as<std::string>().c_str());
        ROS_INFO("Frame ID: %s", map["frame_id"].as<std::string>().c_str());
        ROS_INFO("Number of landmarks: %d", map["num_landmarks"].as<int>());

        ros::Duration(5.0).sleep(); // sleep for 5 seconds

        // load the landmarks in the map object
        for (const auto& node : map["landmarks"])
        {
            Landmark lm;
            lm.id = node["id"].as<int>();
            lm.landmark_pose_.position.x = node["position"][0].as<double>();
            lm.landmark_pose_.position.y = node["position"][1].as<double>();
            lm.landmark_pose_.position.z = node["position"][2].as<double>();
            lm.landmark_pose_.position.z = 0.0; // set the z position to 0.0 - ground robot

            std::vector<double> rvec = node["orientation"].as<std::vector<double>>();
            tf2::Quaternion quaternion;
            quaternion.setRPY(rvec[0], rvec[1], rvec[2]);
            lm.landmark_pose_.orientation.x = quaternion.x();
            lm.landmark_pose_.orientation.y = quaternion.y();
            lm.landmark_pose_.orientation.z = quaternion.z();
            lm.landmark_pose_.orientation.w = quaternion.w();

            mapData.landmarks_.push_back(lm);
        }

        // print the loaded landmarks
        for (const auto& lm : mapData.landmarks_)
        {
            ROS_INFO("Landmark ID: %d", lm.id);
            ROS_INFO("Landmark Position: (%.2f, %.2f, %.2f)", lm.landmark_pose_.position.x, lm.landmark_pose_.position.y, lm.landmark_pose_.position.z);
            ROS_INFO("Landmark Orientation: (%.2f, %.2f, %.2f)", lm.landmark_pose_.orientation.x, lm.landmark_pose_.orientation.y, lm.landmark_pose_.orientation.z);
        }

        // sleep for 5 seconds
        ros::Duration(5.0).sleep();
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR("Error loading map file: %s", e.what());
        ROS_ERROR("Exiting...");
        return 1;
    }

    // -------------------------------------
    // BROADCAST THE LANDMARK POSES
    //
    // broadcast to tf_static the poses of the landmarks in the map file
    //

    // Store the transforms from the markers on the map to the world frame
    std::vector<geometry_msgs::TransformStamped> markers_2_world;
    try
        {
            // broadcast each landmark pose from the map file as a tf2_static transform
            for (int i = 0; i < mapData.num_landmarks_; i++)
            {
                // Create a quaternion from the rotation angles
                tf2::Quaternion quaternion;
                quaternion.setX(mapData.landmarks_[i].landmark_pose_.orientation.x);
                quaternion.setY(mapData.landmarks_[i].landmark_pose_.orientation.y);
                quaternion.setZ(mapData.landmarks_[i].landmark_pose_.orientation.z);
                quaternion.setW(mapData.landmarks_[i].landmark_pose_.orientation.w);

                // configure the message to be broadcasted
                geometry_msgs::TransformStamped marker2world;

                marker2world.header.stamp = ros::Time::now();
                marker2world.header.frame_id = mapData.frame_id_;
                marker2world.child_frame_id = "landmark_" + std::to_string(mapData.landmarks_[i].id);
                marker2world.transform.translation.x = mapData.landmarks_[i].landmark_pose_.position.x;
                marker2world.transform.translation.y = mapData.landmarks_[i].landmark_pose_.position.y;
                marker2world.transform.translation.z = mapData.landmarks_[i].landmark_pose_.position.z;
                marker2world.transform.rotation.x = quaternion.x();
                marker2world.transform.rotation.y = quaternion.y();
                marker2world.transform.rotation.z = quaternion.z();
                marker2world.transform.rotation.w = quaternion.w();

                // add the marker2world transform to the vector of transforms
                markers_2_world.push_back(marker2world);

                ROS_INFO("Broadcasting Marker Pose to TF...");
                ROS_INFO("landmark_%d -> %s", mapData.landmarks_[i].id, mapData.frame_id_.c_str());
            }

            // Broadcast all the landmarks transforms
            static_tf_broadcaster.sendTransform(markers_2_world);

            ROS_INFO("Broadcasted all the map landmarks to TF...");

            // sleep for 5 seconds
            ros::Duration(5.0).sleep();

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }


    // -------------------------------------
    // ROS LOOP
    //
    //

    // Frequency for the loop, in Hertz
    double loop_frequency = 30.0;    /* Hz */
    ros::Rate rate(loop_frequency);

    ROS_WARN("Starting the Localization Node Loop with frequency %f...", loop_frequency);
    ros::Duration(1).sleep();
    ROS_WARN(".");
    ros::Duration(1).sleep();
    ROS_WARN("..");
    ros::Duration(1).sleep();
    ROS_WARN("...");

    while (nh.ok()) {

        // print the time of the loop
        ROS_INFO_STREAM("Loop Time: " << rate.cycleTime());

        // tf2 message structure to store the Transform from detected marker to robot
        geometry_msgs::TransformStamped tf_msg_marker_to_robot;

        /*
         * for each marker, check if the transform exists. if exists, get the transform from the detected markers
         * (marker_frame_ID) to the robot (base_link) and calculate the robot's pose in the world frame, given
         * the map file with the locations of each marker in the world frame
         */
        for (int i = 0; i < mapData.num_landmarks_; i++)
        {
            ROS_WARN("Checking Marker %d...", mapData.landmarks_[i].id);

            // save the detected marker frame name for the current landmark
            std::string marker_frame = "marker_frame_" + std::to_string(mapData.landmarks_[i].id);

            ROS_WARN_STREAM(marker_frame << " -> " << base_frame);

            try
            {
                // Check if the transform exists
                // define a pointer to a std::string to store the error message
                std::string *error_msg = new std::string();

                bool canTransform = tfBuffer.canTransform(base_frame, marker_frame, ros::Time(0), error_msg);

                std::string str_bool = canTransform ? "true" : "false";
                ROS_WARN_STREAM("Can Transform: " << str_bool);
                ROS_ERROR("Error Message: %s", error_msg->c_str());

                // If the transform exists, get the transform from marker to robot
                if (canTransform)
                {
                    // get the transform from detected marker to robot
                    tf_msg_marker_to_robot = tfBuffer.lookupTransform(base_frame, marker_frame, ros::Time(0));

                    // set 'z' to zero because the robot is on the ground
                    tf_msg_marker_to_robot.transform.translation.z = 0.0;

                    // use the pose of the marker in the world/map frame and the transform [Marker->Robot]
                    // to calculate the robot's pose in the world frame
                    //
                    // TF_Marker_2_World = TF_Marker_2_Robot * TF_Robot_2_World
                    //                  |
                    //                  V
                    // TF_Robot_2_World = [TF_Marker_2_Robot]^-1 * [TF_Marker_2_Wworld]
                    tf2::Transform tf_marker_to_robot;
                    tf2::fromMsg(tf_msg_marker_to_robot.transform, tf_marker_to_robot);

                    tf2::Transform tf_marker_to_world;
                    tf2::fromMsg(markers_2_world[i].transform, tf_marker_to_world);

                    // Calculate the robot's pose in the world frame
                    // TF_Robot_2_World = [TF_Marker_2_Robot]^-1 * [TF_Marker_2_World]
                    tf2::Transform tf_robot_to_world;
                    tf_robot_to_world.mult(tf_marker_to_robot.inverse(), tf_marker_to_world);

                    // Broadcast the pose of the robot - the transform from the robot to the world frame
                    geometry_msgs::TransformStamped tf_msg_robot_to_world;
                    tf_msg_robot_to_world.header.stamp = ros::Time::now();
                    tf_msg_robot_to_world.header.frame_id = mapData.frame_id_/* + std::to_string(mapData.landmarks_[i].id)*/;  // world frame from map
                    tf_msg_robot_to_world.child_frame_id = base_frame;  // robot's frame
                    tf_msg_robot_to_world.transform = tf2::toMsg(tf_robot_to_world);


                    // ROS_WARN_STREAM("Robot Pose in World Frame: " << tf_msg_robot_to_world.transform.translation.x << ", " << tf_msg_robot_to_world.transform.translation.y << ", " << tf_msg_robot_to_world.transform.translation.z);

                    tf_broadcaster.sendTransform(tf_msg_robot_to_world);


                    // Convert transform to pose to publish in the topic 'robot_pose'
                    geometry_msgs::PoseStamped pose_message;
                    pose_message.pose.position.x = tf_msg_robot_to_world.transform.translation.x;
                    pose_message.pose.position.y = tf_msg_robot_to_world.transform.translation.y;
                    pose_message.pose.position.z = tf_msg_robot_to_world.transform.translation.z;
                    pose_message.pose.orientation = tf_msg_robot_to_world.transform.rotation;

                    // Set the header of the pose
                    pose_message.header.stamp = ros::Time::now();
                    pose_message.header.frame_id = "base_link";

                    // Publish the pose
                    pose_pub.publish(pose_message);
                }

            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

        }
        rate.sleep();
    }


    return 0;
}