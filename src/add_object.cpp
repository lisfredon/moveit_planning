#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include "moveit_planning/add_object.h"

void addCubeToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                    const std::string& cube_id,
                    const geometry_msgs::Pose& cube_pose,
                    const std::vector<double>& cube_size,
                    const std::string& frame_id)
{
    // Créer l'objet collision
    moveit_msgs::CollisionObject cube;
    cube.header.frame_id = frame_id;
    cube.id = cube_id;

    // Définir le type et les dimensions
    shape_msgs::SolidPrimitive cube_primitive;
    cube_primitive.type = shape_msgs::SolidPrimitive::BOX;
    cube_primitive.dimensions = cube_size;

    // Ajouter la primitive et sa pose
    cube.primitives.push_back(cube_primitive);
    cube.primitive_poses.push_back(cube_pose);
    cube.operation = moveit_msgs::CollisionObject::ADD;

    // Appliquer à la scène
    planning_scene_interface.applyCollisionObjects({cube});
}