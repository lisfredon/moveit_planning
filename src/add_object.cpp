#include "moveit_planning/add_object.h"

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
