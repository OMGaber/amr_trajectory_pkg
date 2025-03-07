#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <amr_trajectory_pkg/SaveTrajectory.h>

struct PoseData {
    geometry_msgs::PoseStamped pose;
    ros::Time timestamp;
};

std::vector<PoseData> trajectory;
ros::Publisher marker_pub;

// Function to save trajectory to a file in different formats
bool saveTrajectory(amr_trajectory_pkg::SaveTrajectory::Request &req,
                    amr_trajectory_pkg::SaveTrajectory::Response &res) {
    std::string filename = req.filename;
    float duration = req.duration;
    std::string format = req.format;
    ros::Time current_time = ros::Time::now();

    // Filter trajectory based on the requested duration
    std::vector<PoseData> filtered_trajectory;
    for (auto it = trajectory.rbegin(); it != trajectory.rend(); ++it) {
        if ((current_time - it->timestamp).toSec() <= duration) {
            filtered_trajectory.push_back(*it);
        } else {
            break;
        }
    }

    std::reverse(filtered_trajectory.begin(), filtered_trajectory.end());

    std::ofstream file(filename);
    if (file.is_open()) {
        if (format == "csv") {
            file << "timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w\n";
            for (const auto &data : filtered_trajectory) {
                const auto &pose = data.pose.pose;
                file << data.timestamp.toSec() << ","
                     << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
                     << pose.orientation.x << "," << pose.orientation.y << ","
                     << pose.orientation.z << "," << pose.orientation.w << "\n";
            }
        } else if (format == "json") {
            file << "[\n";
            for (size_t i = 0; i < filtered_trajectory.size(); ++i) {
                const auto &pose = filtered_trajectory[i].pose.pose;
                file << "  {\n";
                file << "    \"timestamp\": " << filtered_trajectory[i].timestamp.toSec() << ",\n";
                file << "    \"position\": {\"x\": " << pose.position.x << ", \"y\": " << pose.position.y << ", \"z\": " << pose.position.z << "},\n";
                file << "    \"orientation\": {\"x\": " << pose.orientation.x << ", \"y\": " << pose.orientation.y << ", \"z\": " << pose.orientation.z << ", \"w\": " << pose.orientation.w << "}\n";
                file << "  }" << (i == filtered_trajectory.size() - 1 ? "\n" : ",\n");
            }
            file << "]\n";
        } else if (format == "yaml") {
            for (const auto &data : filtered_trajectory) {
                const auto &pose = data.pose.pose;
                file << "- timestamp: " << data.timestamp.toSec() << "\n";
                file << "  position:\n";
                file << "    x: " << pose.position.x << "\n";
                file << "    y: " << pose.position.y << "\n";
                file << "    z: " << pose.position.z << "\n";
                file << "  orientation:\n";
                file << "    x: " << pose.orientation.x << "\n";
                file << "    y: " << pose.orientation.y << "\n";
                file << "    z: " << pose.orientation.z << "\n";
                file << "    w: " << pose.orientation.w << "\n";
            }
        } else {
            res.success = false;
            res.message = "Unsupported file format.";
            ROS_ERROR("Unsupported file format: %s", format.c_str());
            return true;
        }

        file.close();
        res.success = true;
        res.message = "Trajectory saved successfully.";
        ROS_INFO("Trajectory saved to %s", filename.c_str());
    } else {
        res.success = false;
        res.message = "Failed to open file.";
        ROS_ERROR("Failed to open file: %s", filename.c_str());
    }
    return true;
}

// Callback to store the robot's trajectory
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    PoseData data;
    data.pose.header = msg->header;
    data.pose.pose = msg->pose.pose;  // Extract pose without covariance
    data.timestamp = ros::Time::now();
    trajectory.push_back(data);

    // Create and publish marker for visualization
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.id = trajectory.size();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = msg->pose.pose;  // Extract only the Pose part here
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_pub.publish(marker_array);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_saver_node");
    ros::NodeHandle nh;

    // Subscriber to get the robot's pose
    ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 1000, poseCallback);

    // Publisher for RViz markers
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 10);

    // Service to save trajectory
    ros::ServiceServer service = nh.advertiseService("save_trajectory", saveTrajectory);

    ROS_INFO("Trajectory Saver Node is running...");
    ros::spin();

    return 0;
}
