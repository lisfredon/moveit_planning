#include "moveit_planning/approch_utils.h"
#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/transport_utils.h"
#include "moveit_planning/retreat_utils.h"
#include "moveit_planning/solvers_utils.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <iostream>
#include <string>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <vector>

SolverType solverFromString(const std::string& s) {
    if (s == "OMPL") return SolverType::OMPL;
    else if (s == "CARTESIAN") return SolverType::CARTESIAN;
    else if (s == "JOINT_INTERPOLATION") return SolverType::JOINT_INTERPOLATION;
    else {
        std::cerr << "Solver inconnu '" << s << "', choix par défaut OMPL" << std::endl;
        return SolverType::OMPL;
    }
}

geometry_msgs::Pose loadParam(const std::string& param_namespace) {
    std::vector<double> pos, ori;
    geometry_msgs::Pose pose;
    if (!ros::param::get(param_namespace + "/position", pos) ||
        !ros::param::get(param_namespace + "/orientation", ori)) {
        ROS_ERROR_STREAM("Impossible de charger les paramètres pour " << param_namespace);
        throw std::runtime_error("Paramètres manquants");
    }
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];
    return pose;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pseudo_mtc");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Charger poses et tailles
    std::vector<double> cube_size;
        if (!ros::param::get("/cube/size", cube_size) || cube_size.size() != 3) {
            ROS_ERROR("Impossible de charger la taille du cube !");
            return 1;
        }
        geometry_msgs::Pose object_pose = loadParam("/cube");

        std::vector<double> table_size;
        if (!ros::param::get("/table/size", table_size) || table_size.size() != 3) {
            ROS_ERROR("Impossible de charger la taille de la table !");
            return 1;
        }
        geometry_msgs::Pose table_pose = loadParam("/table");

        geometry_msgs::Pose goal_pose = loadParam("/goal");

        // Ajouter cube
        moveit_msgs::CollisionObject cube;
        cube.header.frame_id = "panda_link0";
        cube.id = "cube";

        shape_msgs::SolidPrimitive cube_primitive;
        cube_primitive.type = shape_msgs::SolidPrimitive::BOX;
        cube_primitive.dimensions = cube_size;

        cube.primitives.push_back(cube_primitive);
        cube.primitive_poses.push_back(object_pose);
        cube.operation = moveit_msgs::CollisionObject::ADD;

        planning_scene_interface.applyCollisionObjects({cube});

        // Ajouter table
        moveit_msgs::CollisionObject table;
        table.header.frame_id = "panda_link0";
        table.id = "table";

        shape_msgs::SolidPrimitive table_primitive;
        table_primitive.type = shape_msgs::SolidPrimitive::BOX;
        table_primitive.dimensions = table_size;

        table.primitives.push_back(table_primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = moveit_msgs::CollisionObject::ADD;

        planning_scene_interface.applyCollisionObjects({table});
    
        move_group.setPlanningTime(10.0); 


    // Choix du type d'approche
    std::string approach_str;
    std::cout << "Type d'approche (verticale/horizontale/diagonale): ";
    std::cin >> approach_str;
    ApproachType approach_type = approachTypeFromString(approach_str);
    geometry_msgs::Pose approach_pose = createApproachPose(object_pose, approach_type, 0.1);

    /*
    // Liste des stratégies possibles
    std::vector<std::string> grasp_strategies = {"F_verticale", "F_horizontale", "F_droite", "F_gauche", "H_perpendiculaire", "H_parallele"};

    geometry_msgs::Pose grasp_pose;
    bool grasp_found = false;

    for (const auto& strategy : grasp_strategies) {
        grasp_pose = createGraspPose(object_pose, strategy);
        ROS_INFO_STREAM("Essai de saisie avec stratégie: " << strategy);

        // Test de planification sans exécuter
        move_group.setPoseTarget(grasp_pose);
        moveit::planning_interface::MoveGroupInterface::Plan test_plan;

        if (move_group.plan(test_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO_STREAM("Stratégie " << strategy << " valide !");
            grasp_found = true;
            break; // On garde cette stratégie
        } else {
            ROS_WARN_STREAM("Stratégie " << strategy << " non atteignable, on essaie la suivante...");
        }
    }
    

        // --- Génération automatique des poses de saisie ---
    tf2::Quaternion default_grasp_orientation;
    default_grasp_orientation.setRPY(0.0, 0.0, 0.0);// exemple : pince vers l'avant

    auto grasp_candidates = generateGraspPoses(object_pose, Eigen::Vector3d::UnitZ(), M_PI / 12.0, 2 * M_PI, default_grasp_orientation);

    geometry_msgs::Pose grasp_pose;
    bool grasp_found = false;

    for (size_t i = 0; i < grasp_candidates.size(); ++i) {
        const geometry_msgs::Pose& pose = grasp_candidates[i];

        // Affichage de la position
        ROS_INFO_STREAM("Pose #" << i << " position: ["
                        << pose.position.x << ", "
                        << pose.position.y << ", "
                        << pose.position.z << "]");

        // Convertir orientation quaternion en RPY pour plus de lisibilité
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO_STREAM("Pose #" << i << " orientation (RPY): ["
                        << roll << ", "
                        << pitch << ", "
                        << yaw << "]");

        // Ensuite tu peux faire ta planification
        move_group.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan test_plan;

        if (move_group.plan(test_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO_STREAM("Pose #" << i << " atteignable, saisie retenue !");
            grasp_pose = pose;
            grasp_found = true;
            break;
        } else {
            ROS_WARN_STREAM("Pose #" << i << " non atteignable.");
        }
    }
    */

    //ici
    auto grasp_candidates = generateFaceGraspPoses(object_pose);

    geometry_msgs::Pose grasp_pose;
    bool grasp_found = false;

    ros::NodeHandle nh;
    ros::Publisher pose_attempt_pub = nh.advertise<geometry_msgs::PoseStamped>("grasp_pose_attempt", 10);

    for (size_t i = 0; i < grasp_candidates.size(); ++i) {
        ROS_INFO_STREAM("Essai de saisie avec pose #" << i);
        ROS_INFO_STREAM("Position: [" << grasp_candidates[i].position.x << ", "
                                    << grasp_candidates[i].position.y << ", "
                                    << grasp_candidates[i].position.z << "]");
        tf2::Quaternion q(
            grasp_candidates[i].orientation.x,
            grasp_candidates[i].orientation.y,
            grasp_candidates[i].orientation.z,
            grasp_candidates[i].orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("Orientation RPY: [" << roll << ", " << pitch << ", " << yaw << "]");
        Eigen::Quaterniond eq(q.w(), q.x(), q.y(), q.z());
        Eigen::Matrix3d R = eq.toRotationMatrix();
        ROS_INFO_STREAM("X: " << R.col(0).transpose());
        ROS_INFO_STREAM("Y: " << R.col(1).transpose());
        ROS_INFO_STREAM("Z: " << R.col(2).transpose()); 

        // Création du message PoseStamped
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "panda_link0";  // adapte selon ton frame fixe (souvent base_link ou panda_link0)
        pose_msg.pose = grasp_candidates[i];

        // Publier la pose
        pose_attempt_pub.publish(pose_msg);

        // Petit délai pour que RViz ait le temps d’afficher (optionnel)
        ros::Duration(0.1).sleep();

        move_group.setPoseTarget(grasp_candidates[i]);
        moveit::planning_interface::MoveGroupInterface::Plan test_plan;

        if (move_group.plan(test_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO_STREAM("Pose #" << i << " atteignable !");
            grasp_pose = grasp_candidates[i];
            grasp_found = true;
            break;
        } else {
            ROS_WARN_STREAM("Pose #" << i << " non atteignable.");
        }
    }

    //ici 


    if (!grasp_found) {
        ROS_ERROR("Aucune stratégie de saisie n'est atteignable !");
        return 1;
    }

    geometry_msgs::Pose transport_pose = createTransportPose(grasp_pose, goal_pose);
    geometry_msgs::Pose retreat_pose = createRetreatPose(grasp_pose, 0.1);

    // Choix des solveurs pour chaque étape
    std::string s_approach_solver, s_grasp_solver, s_transport_solver, s_retreat_solver;

    std::cout << "Solver pour l'approche (OMPL / CARTESIAN / JOINT_INTERPOLATION): ";
    std::cin >> s_approach_solver;

    std::cout << "Solver pour la saisie (OMPL / CARTESIAN / JOINT_INTERPOLATION): ";
    std::cin >> s_grasp_solver;

    std::cout << "Solver pour le transport (OMPL / CARTESIAN / JOINT_INTERPOLATION): ";
    std::cin >> s_transport_solver;

    std::cout << "Solver pour le retrait (OMPL / CARTESIAN / JOINT_INTERPOLATION): ";
    std::cin >> s_retreat_solver;

    // Conversion string -> enum
    SolverType approach_solver = solverFromString(s_approach_solver);
    SolverType grasp_solver = solverFromString(s_grasp_solver);
    SolverType transport_solver = solverFromString(s_transport_solver);
    SolverType retreat_solver = solverFromString(s_retreat_solver);

    // --- Exécution ---
    if (!planAndExecute(move_group, approach_pose, approach_solver)) {
        ROS_ERROR("Échec de l'approche");
        return 1;
    }
    ROS_INFO("reussi 1");

    if (!planAndExecute(move_group, grasp_pose, grasp_solver)) {
        ROS_ERROR("Échec de la saisie même après sélection d'une stratégie valide.");
        return 1;
    }
    ROS_INFO("reussi 2");
    // --- Attacher l'objet au robot ---
    move_group.attachObject("cube", "panda_hand");
    ROS_INFO("Cube attaché au gripper.");

    if (!planAndExecute(move_group, transport_pose, transport_solver)) {
        ROS_ERROR("Échec du transport");
        return 1;
    }

    if (!planAndExecute(move_group, retreat_pose, retreat_solver)) {
        ROS_ERROR("Échec du retrait");
        return 1;
    }
        // --- Détacher l'objet ---
    move_group.detachObject("cube");
    ROS_INFO("Cube détaché du gripper.");

    ros::shutdown();
    return 0;
}
