#include "moveit_planning/visualization_utils.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void publishFaceNormalsWithText(const geometry_msgs::Pose& cube_pose,
                                const std::vector<tf2::Vector3>& face_normals,
                                const std::vector<std::string>& face_names,
                                ros::Publisher& marker_pub) {
    visualization_msgs::MarkerArray marker_array;

    tf2::Quaternion q_cube;
    tf2::fromMsg(cube_pose.orientation, q_cube);
    tf2::Matrix3x3 R_cube(q_cube);
    tf2::Vector3 p_cube(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);

    for (size_t i = 0; i < face_normals.size(); ++i) {
        tf2::Vector3 n_global = R_cube * face_normals[i];

        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "panda_link0";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "face_normals";
        arrow.id = i;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start, end;
        start.x = p_cube.x();
        start.y = p_cube.y();
        start.z = p_cube.z();
        end.x = p_cube.x() + 0.1 * n_global.x();
        end.y = p_cube.y() + 0.1 * n_global.y();
        end.z = p_cube.z() + 0.1 * n_global.z();
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        arrow.scale.x = 0.01;
        arrow.scale.y = 0.02;
        arrow.color.a = 1.0;

        if (face_names[i] == "+X") { arrow.color.r = 1.0; }
        else if (face_names[i] == "-X") { arrow.color.r = 0.5; }
        else if (face_names[i] == "+Y") { arrow.color.g = 1.0; }
        else if (face_names[i] == "-Y") { arrow.color.g = 0.5; }
        else if (face_names[i] == "+Z") { arrow.color.b = 1.0; }
        else if (face_names[i] == "-Z") { arrow.color.b = 0.5; }

        marker_array.markers.push_back(arrow);

        visualization_msgs::Marker text;
        text.header.frame_id = "panda_link0";
        text.header.stamp = ros::Time::now();
        text.ns = "face_normals_text";
        text.id = i + 100;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.pose.position = end;
        text.scale.z = 0.05;
        text.color.r = text.color.g = text.color.b = 1.0;
        text.color.a = 1.0;
        text.text = face_names[i];
        marker_array.markers.push_back(text);
    }

    marker_pub.publish(marker_array);
}
