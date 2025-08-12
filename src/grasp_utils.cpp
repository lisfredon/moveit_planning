/*
#include <ros/ros.h>

#include "moveit_planning/grasp_utils.h"
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::Pose createGraspPose(const geometry_msgs::Pose& object_pose,
                                    const std::string& strategy) {
    geometry_msgs::Pose grasp_pose = object_pose;
    tf2::Quaternion q;

    if (strategy == "F_verticale") {
        q.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    } else if (strategy == "F_horizontale") {
        q.setRPY(0.0, M_PI / 2, 0.0);
    } else if (strategy == "F_droite") {
        q.setRPY(0.0, -M_PI / 2, -M_PI / 2);  
    } else if (strategy == "F_gauche") {
        q.setRPY(0.0, -M_PI / 2, M_PI / 2);
    } else if (strategy == "H_perpendiculaire") {
        q.setRPY(-M_PI, 0.0, -M_PI / 2);
    } else if (strategy == "H_parallele") {
        q.setRPY(-M_PI, 0.0, 0.0);  
    } else {
        q.setRPY(0.0, 0.0, 0.0); //prend par le bas
    }

    grasp_pose.orientation.x = q.x();
    grasp_pose.orientation.y = q.y();
    grasp_pose.orientation.z = q.z();
    grasp_pose.orientation.w = q.w();
    return grasp_pose;
}



#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h> 
#include <Eigen/Geometry>
#include <vector>
#include "moveit_planning/grasp_utils.h"

/// Génère plusieurs poses de préhension en tournant autour d'un axe
std::vector<geometry_msgs::Pose> generateGraspPoses(
    const geometry_msgs::Pose& object_pose,
    const Eigen::Vector3d& rotation_axis = Eigen::Vector3d::UnitZ(),
    double angle_delta = M_PI / 8.0,
    double max_angle = 2 * M_PI,
    const tf2::Quaternion& orientation_offset = tf2::Quaternion(0, 0, 0, 1)) {

    std::vector<geometry_msgs::Pose> grasp_poses;

    double current_angle = 0.0;
    while (std::abs(current_angle) <= std::abs(max_angle)) {
        // Rotation autour de l'axe
        Eigen::AngleAxisd rotation(current_angle, rotation_axis.normalized());
        Eigen::Isometry3d grasp_pose_eigen = Eigen::Isometry3d::Identity();
        grasp_pose_eigen.linear() = rotation.toRotationMatrix();

        // Appliquer la rotation à l'orientation existante
        tf2::Transform tf_grasp;
        tf2::fromMsg(object_pose, tf_grasp);

        Eigen::Affine3d final_tf = Eigen::Translation3d(object_pose.position.x, object_pose.position.y, object_pose.position.z) * rotation;
        geometry_msgs::Pose new_pose;
        new_pose = tf2::toMsg(final_tf);


        // Appliquer un offset supplémentaire si voulu (ex: orientation de la pince par défaut)
        tf2::Quaternion current_q;
        tf2::fromMsg(new_pose.orientation, current_q);
        current_q *= orientation_offset;
        current_q.normalize();

        new_pose.orientation = tf2::toMsg(current_q);
        new_pose.position = object_pose.position;

        grasp_poses.push_back(new_pose);
        current_angle += angle_delta;
    }

    return grasp_poses;
}



#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h> 
#include <tf2_eigen/tf2_eigen.h>
#include "moveit_planning/grasp_utils.h"
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <vector>

std::vector<geometry_msgs::Pose> generateFaceGraspPoses(const geometry_msgs::Pose& cube_pose_msg)
{
    std::vector<geometry_msgs::Pose> grasp_poses;

    // Convert to Eigen for easier manipulation
    Eigen::Isometry3d cube_tf;
    tf2::fromMsg(cube_pose_msg, cube_tf);

    // Normals of each face (in local frame)
    std::vector<Eigen::Vector3d> face_normals = {
        Eigen::Vector3d::UnitX(),   // +X
        -Eigen::Vector3d::UnitX(),  // -X
        Eigen::Vector3d::UnitY(),   // +Y
        -Eigen::Vector3d::UnitY(),  // -Y
        Eigen::Vector3d::UnitZ(),   // +Z (dessus)
        -Eigen::Vector3d::UnitZ()   // -Z (dessous)
    };

    for (const auto& normal_local : face_normals)
    {
        // Transform normal to world frame
        Eigen::Vector3d approach_dir = cube_tf.rotation() * normal_local;

        // Définir un axe "haut" pour construire la base
        Eigen::Vector3d up_dir(0, 0, 1);
        if (fabs(approach_dir.dot(up_dir)) > 0.95)  // Trop proche
            up_dir = Eigen::Vector3d(1, 0, 0);

        // Construire une rotation orthonormée
        Eigen::Vector3d x_axis = up_dir.cross(approach_dir).normalized();
        Eigen::Vector3d y_axis = approach_dir.cross(x_axis).normalized();

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix.col(0) = x_axis;
        rotation_matrix.col(1) = y_axis;
        rotation_matrix.col(2) = approach_dir;

        Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
        grasp_tf.linear() = rotation_matrix;
        double offset = 0.05/ 2.0 + 0.1; // moitié de la taille du cube + marge (10 cm ici)
        grasp_tf.translation() = cube_tf.translation() + approach_dir.normalized() * offset;
        //grasp_tf.translation() = cube_tf.translation();

        geometry_msgs::Pose grasp_pose = tf2::toMsg(grasp_tf);
        grasp_poses.push_back(grasp_pose);
    }

    return grasp_poses;
}

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>

std::vector<geometry_msgs::Pose> generateFaceGraspPoses(
    const geometry_msgs::Pose& cube_pose_msg)
{
    std::vector<geometry_msgs::Pose> poses;

    const double cube_size = 0.05;          // taille du cube (modifiable si besoin)
    const double approach_distance = 0.10;  // distance d'approche avant la prise
    const double rotation_step_deg = 15.0;  // rotation autour de l'axe d'approche pour varier

    // Conversion quaternion ROS -> tf2
    tf2::Quaternion q_cube;
    tf2::fromMsg(cube_pose_msg.orientation, q_cube);
    tf2::Matrix3x3 rot_cube(q_cube);

    // Normales locales des faces du cube (dans son repère)
    std::vector<tf2::Vector3> face_normals = {
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1}
    };

    tf2::Vector3 cube_center(
        cube_pose_msg.position.x,
        cube_pose_msg.position.y,
        cube_pose_msg.position.z);

    for (const auto& normal_local : face_normals)
    {
        // Normale de face en coordonnées mondiales
        tf2::Vector3 normal_world = rot_cube * normal_local;

        // Point de prise : centre + demi taille * normale
        tf2::Vector3 grasp_point = cube_center + normal_world * (cube_size / 2.0);

        // Direction d'approche = inverse de la normale
        tf2::Vector3 approach_dir = -normal_world.normalized();

        // Axe "up" = axe Z local du cube, sauf si presque colinéaire avec approche
        tf2::Vector3 up_world = rot_cube * tf2::Vector3(0, 0, 1);
        if (fabs(up_world.dot(approach_dir)) > 0.9)
            up_world = rot_cube * tf2::Vector3(0, 1, 0);

        // Calcul des axes X et Y pour orientation
        tf2::Vector3 x_axis = up_world.cross(approach_dir).normalized();
        tf2::Vector3 y_axis = approach_dir.cross(x_axis).normalized();

        tf2::Matrix3x3 rot_tool(
            x_axis.x(), y_axis.x(), approach_dir.x(),
            x_axis.y(), y_axis.y(), approach_dir.y(),
            x_axis.z(), y_axis.z(), approach_dir.z());

        tf2::Quaternion q_tool;
        rot_tool.getRotation(q_tool);

        // Générer plusieurs orientations en tournant autour de l'axe d'approche
        for (double rot_deg = -rotation_step_deg; rot_deg <= rotation_step_deg; rot_deg += rotation_step_deg)
        {
            tf2::Quaternion q_rot;
            q_rot.setRotation(approach_dir, rot_deg * M_PI / 180.0);
            tf2::Quaternion q_final = q_rot * q_tool;

            geometry_msgs::Pose approach_pose;
            approach_pose.position.x = grasp_point.x() + approach_dir.x() * approach_distance;
            approach_pose.position.y = grasp_point.y() + approach_dir.y() * approach_distance;
            approach_pose.position.z = grasp_point.z() + approach_dir.z() * approach_distance;
            approach_pose.orientation = tf2::toMsg(q_final);

            // Position finale de la saisie (au contact)
            geometry_msgs::Pose grasp_pose = approach_pose;
            grasp_pose.position.x -= approach_dir.x() * approach_distance;
            grasp_pose.position.y -= approach_dir.y() * approach_distance;
            grasp_pose.position.z -= approach_dir.z() * approach_distance;

            poses.push_back(approach_pose);
            poses.push_back(grasp_pose);
        }
    }

    return poses;
}
*/
#include <vector>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>  // pour fabs

std::vector<geometry_msgs::Pose> generateFaceGraspPoses(
    const geometry_msgs::Pose& cube_pose_msg)
{
    std::vector<geometry_msgs::Pose> grasp_poses;

    double half_size = 0.05; // 5cm demi taille cube
    double grasp_offset = 0.05; // 5cm devant la face

    // Normales des faces locales du cube
    std::vector<tf2::Vector3> face_normals = {
        tf2::Vector3(1, 0, 0),
        tf2::Vector3(-1, 0, 0),
        tf2::Vector3(0, 1, 0),
        tf2::Vector3(0, -1, 0),
        tf2::Vector3(0, 0, 1),
        tf2::Vector3(0, 0, -1)
    };

    // Conversion pose du cube en tf2::Transform
    tf2::Transform cube_tf;
    tf2::fromMsg(cube_pose_msg, cube_tf);

    for (const auto& normal_local : face_normals)
    {
        // Position locale devant la face
        tf2::Vector3 pos_local = normal_local * (half_size + grasp_offset);

        // Position dans le monde
        tf2::Vector3 pos_world = cube_tf * pos_local;

        // Direction Z de la préhension = normale dans repère monde
        tf2::Vector3 z_axis = (cube_tf.getBasis() * normal_local).normalized();

        // Choix d'un axe X arbitraire perpendiculaire à z_axis
        tf2::Vector3 arbitrary_axis;
        if (fabs(z_axis.dot(tf2::Vector3(1, 0, 0))) < 0.9)
            arbitrary_axis = tf2::Vector3(1, 0, 0);
        else
            arbitrary_axis = tf2::Vector3(0, 1, 0);

        // Calcul des axes X et Y de la base orthonormée
        tf2::Vector3 x_axis = arbitrary_axis.cross(z_axis).normalized();
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();

        // Construction de la matrice rotation : les colonnes sont x, y, z
        tf2::Matrix3x3 rotation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        // Correction: La matrice tf2::Matrix3x3 attend les éléments ligne par ligne :
        // m00, m01, m02,
        // m10, m11, m12,
        // m20, m21, m22

        // Donc il faut faire :

        rotation_matrix = tf2::Matrix3x3(
            x_axis.x(), x_axis.y(), x_axis.z(),
            y_axis.x(), y_axis.y(), y_axis.z(),
            z_axis.x(), z_axis.y(), z_axis.z());

        tf2::Quaternion quat;
        rotation_matrix.getRotation(quat);

        geometry_msgs::Pose grasp_pose;
        grasp_pose.position.x = pos_world.x();
        grasp_pose.position.y = pos_world.y();
        grasp_pose.position.z = pos_world.z();
        grasp_pose.orientation = tf2::toMsg(quat);

        grasp_poses.push_back(grasp_pose);
    }

    return grasp_poses;
}
