#include "moveit_planning/load_add_object.h"

// Charger une pose depuis le paramètre ROS
geometry_msgs::Pose loadObjectPose(const std::string& param_namespace) {
    std::vector<double> pos, ori;
    geometry_msgs::Pose pose;

    bool pos_ok = ros::param::get(param_namespace + "/position", pos);
    bool ori_ok = ros::param::get(param_namespace + "/orientation", ori);

    if (!pos_ok) {
        ROS_ERROR_STREAM("Position manquante pour " << param_namespace);
        throw std::runtime_error("Paramètres manquants");
    }
    if (!ori_ok) {
        ROS_ERROR_STREAM("Orientation manquante pour " << param_namespace);
        throw std::runtime_error("Paramètres manquants");
    }

    // Compléter la position à 3 valeurs avec des 0.0
    if (pos.size() < 3) {
        pos.resize(3, 0.0);
        ROS_WARN_STREAM("Position incomplète pour " << param_namespace
                        << " -> complétée à : [" 
                        << pos[0] << ", " << pos[1] << ", " << pos[2] << "]");
    }

    // Compléter l'orientation à 4 valeurs avec des 0.0 et un w = 1.0 par défaut
    if (ori.size() < 4) {
        while (ori.size() < 3) ori.push_back(0.0);
        if (ori.size() == 3) ori.push_back(1.0); // w par défaut
        ROS_WARN_STREAM("Orientation incomplète pour " << param_namespace
                        << " -> complétée à : [" 
                        << ori[0] << ", " << ori[1] << ", " << ori[2] << ", " << ori[3] << "]");
    }

    // Affecter à la pose
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];

    return pose;
}


std::vector<double> loadObjectSize(const std::string& param_name) {
    std::vector<double> size;
    if (!ros::param::get(param_name, size) || size.size() != 3) {
        ROS_ERROR_STREAM("Impossible de charger la taille du cube : " << param_name);
        return {};
    }
    return size;
}

moveit_msgs::CollisionObject addObjectToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                              const std::string& object_id,
                                              const geometry_msgs::Pose& object_pose,
                                              const std::vector<double>& object_size,
                                              const std::string& frame_id)
{
    // Créer l'objet collision
    moveit_msgs::CollisionObject object;
    object.header.frame_id = frame_id;
    object.id = object_id;

    // Définir le type et les dimensions
    shape_msgs::SolidPrimitive object_primitive;
    object_primitive.type = shape_msgs::SolidPrimitive::BOX;
    object_primitive.dimensions = object_size;

    // Ajouter la primitive et sa pose
    object.primitives.push_back(object_primitive);
    object.primitive_poses.push_back(object_pose);
    object.operation = moveit_msgs::CollisionObject::ADD;

    // Appliquer à la scène
    planning_scene_interface.applyCollisionObjects({object});

    // Retourner l'objet pour pouvoir l'utiliser ailleurs (ex. attacher)
    return object;
}
