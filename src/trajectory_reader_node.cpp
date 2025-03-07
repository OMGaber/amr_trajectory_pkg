#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>        // Include YAML parser
#include <nlohmann/json.hpp>      // Include JSON parser

using json = nlohmann::json;     // Define alias for convenience

ros::Publisher marker_pub;
tf2_ros::Buffer tfBuffer;

// Function to create and publish a marker
void createAndPublishMarker(const geometry_msgs::PoseStamped &pose, int marker_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_pub.publish(marker_array);
}

// Function to parse a single line of CSV
bool parseCSVLine(const std::string &line, double &timestamp, geometry_msgs::Pose &pose) {
    std::stringstream ss(line);
    std::string token;
    try {
        std::getline(ss, token, ','); timestamp = std::stod(token);
        std::getline(ss, token, ','); pose.position.x = std::stod(token);
        std::getline(ss, token, ','); pose.position.y = std::stod(token);
        std::getline(ss, token, ','); pose.position.z = std::stod(token);
        std::getline(ss, token, ','); pose.orientation.x = std::stod(token);
        std::getline(ss, token, ','); pose.orientation.y = std::stod(token);
        std::getline(ss, token, ','); pose.orientation.z = std::stod(token);
        std::getline(ss, token, ','); pose.orientation.w = std::stod(token);
        return true;
    } catch (...) {
        ROS_WARN("Failed to parse CSV line.");
        return false;
    }
}

// Function to parse a single YAML entry
bool parseYAMLEntry(const YAML::Node &entry, double &timestamp, geometry_msgs::Pose &pose) {
    try {
        timestamp = entry["timestamp"].as<double>();
        pose.position.x = entry["position"]["x"].as<double>();
        pose.position.y = entry["position"]["y"].as<double>();
        pose.position.z = entry["position"]["z"].as<double>();
        pose.orientation.x = entry["orientation"]["x"].as<double>();
        pose.orientation.y = entry["orientation"]["y"].as<double>();
        pose.orientation.z = entry["orientation"]["z"].as<double>();
        pose.orientation.w = entry["orientation"]["w"].as<double>();
        return true;
    } catch (...) {
        ROS_WARN("Failed to parse YAML entry.");
        return false;
    }
}

// Function to parse a single JSON entry
bool parseJSONEntry(const json &entry, double &timestamp, geometry_msgs::Pose &pose) {
    try {
        timestamp = entry["timestamp"];
        pose.position.x = entry["position"]["x"];
        pose.position.y = entry["position"]["y"];
        pose.position.z = entry["position"]["z"];
        pose.orientation.x = entry["orientation"]["x"];
        pose.orientation.y = entry["orientation"]["y"];
        pose.orientation.z = entry["orientation"]["z"];
        pose.orientation.w = entry["orientation"]["w"];
        return true;
    } catch (...) {
        ROS_WARN("Failed to parse JSON entry.");
        return false;
    }
}

// Unified function to handle different file formats
bool readAndPublishTrajectory(const std::string &filename, const std::string &format) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return false;
    }

    int marker_id = 0;

    if (format == "yaml") {
        YAML::Node yaml_data = YAML::LoadFile(filename);
        for (const auto &entry : yaml_data) {
            double timestamp;
            geometry_msgs::Pose pose;
            if (!parseYAMLEntry(entry, timestamp, pose)) continue;

            geometry_msgs::PoseStamped pose_in, pose_out;
            pose_in.header.frame_id = "map";
            pose_in.header.stamp = ros::Time(timestamp);
            pose_in.pose = pose;

            try {
                pose_out = tfBuffer.transform(pose_in, "odom", ros::Duration(1.0));
                createAndPublishMarker(pose_out, marker_id++);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Transform failed: %s", ex.what());
                continue;
            }
        }
    } else if (format == "json") {
        json json_data;
        file >> json_data;
        for (const auto &entry : json_data) {
            double timestamp;
            geometry_msgs::Pose pose;
            if (!parseJSONEntry(entry, timestamp, pose)) continue;

            geometry_msgs::PoseStamped pose_in, pose_out;
            pose_in.header.frame_id = "map";
            pose_in.header.stamp = ros::Time(timestamp);
            pose_in.pose = pose;

            try {
                pose_out = tfBuffer.transform(pose_in, "odom", ros::Duration(1.0));
                createAndPublishMarker(pose_out, marker_id++);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Transform failed: %s", ex.what());
                continue;
            }
        }
    } else {
        std::string line;
        std::getline(file, line); // Skip header
        while (std::getline(file, line)) {
            double timestamp;
            geometry_msgs::Pose pose;
            if (!parseCSVLine(line, timestamp, pose)) continue;

            geometry_msgs::PoseStamped pose_in, pose_out;
            pose_in.header.frame_id = "map";
            pose_in.header.stamp = ros::Time(timestamp);
            pose_in.pose = pose;

            try {
                pose_out = tfBuffer.transform(pose_in, "odom", ros::Duration(1.0));
                createAndPublishMarker(pose_out, marker_id++);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Transform failed: %s", ex.what());
                continue;
            }
        }
    }

    file.close();
    ROS_INFO("Trajectory published for visualization.");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_reader_node");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 10);

    tf2_ros::TransformListener tfListener(tfBuffer);

    std::string filename, format;
    nh.getParam("trajectory_file", filename);
    nh.getParam("trajectory_format", format);

    if (!readAndPublishTrajectory(filename, format)) {
        ROS_ERROR("Failed to read and publish trajectory.");
        return 1;
    }

    ros::spin();
    return 0;
}
