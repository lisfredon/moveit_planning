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

std::string loadObjectID(const std::string& param_name) {
    std::string id;
    if (!ros::param::get(param_name, id)) {
        ROS_ERROR_STREAM("Impossible de charger l'ID de l'objet : " << param_name);
        return ""; 
    }
    return id;
}

std::string loadSideFace(const std::string& param_name) {
    std::string side_face;
    if (!ros::param::get(param_name, side_face)) {
        ROS_ERROR_STREAM("Impossible de charger le coté de saisie souhaité : " << param_name);
        return ""; 
    }
    return side_face;
}

int loadGoalFace(const std::string& param_name) {
    int choice_face;
    if (!ros::param::get(param_name, choice_face)) {
        ROS_ERROR_STREAM("Impossible de charger la face de saisie souhaité : " << param_name);
        return {}; 
    }
    return choice_face;
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

tf2::Vector3 getNormalObject(int face_index) {
    static std::vector<tf2::Vector3> normals = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };
    return normals[face_index];
}

tf2::Vector3 getObjectAxis(int face_index, const std::string& side_face) {
    static const std::vector<tf2::Vector3> width_axes = {
        {0,0,1}, {0,0,1}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}
    };
    static const std::vector<tf2::Vector3> length_axes = {
        {0,1,0}, {0,1,0}, {0,0,1}, {0,0,1}, {0,1,0}, {0,1,0}
    };

    if (face_index < 0 || face_index >= static_cast<int>(width_axes.size())) {
        throw std::out_of_range("face_index invalide : doit être entre 0 et 5");
    }

    if (side_face == "width") {
        return width_axes[face_index];
    } else if (side_face == "length") {
        return length_axes[face_index];
    } else {
        throw std::invalid_argument("side_face invalide : doit être 'width' ou 'length'");
    }
}