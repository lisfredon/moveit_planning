
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_planning/PickPlaceAction.h>

#include "moveit_planning/pick_place_manager.h"
#include "moveit_planning/solvers_utils.h"
#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/load_add_object.h"
#include "moveit_planning/visualization_utils.h"
#include "moveit_planning/error_handling.h"

class PickPlaceActionServer {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<moveit_planning::PickPlaceAction> as_;
    std::string action_name_;
    moveit_planning::PickPlaceResult result_;
    moveit_planning::PickPlaceFeedback feedback_;
    PickPlaceManager manager_;

public:
    PickPlaceActionServer(std::string name) :
        as_(nh_, name, boost::bind(&PickPlaceActionServer::executeCB, this, _1), false),
        action_name_(name),
        manager_("panda_manipulator", "panda_hand")
    {
        // --- AJOUTER LES CUBES DANS LA SCÈNE ---
        std::vector<std::string> cube_names;
        if (!nh_.getParam("/cubes", cube_names)) {
            ROS_WARN("Aucun cube trouvé dans le param server (/cubes)");
        } else {
            for (const auto& cube_ns : cube_names) {
                std::string cube_id = loadObjectID("/" + cube_ns + "/id");
                geometry_msgs::Pose cube_pose = loadObjectPose("/" + cube_ns);
                std::vector<double> cube_size = loadObjectSize("/" + cube_ns + "/size");
                manager_.addObject(cube_id, cube_pose, cube_size);
                ROS_INFO("Cube '%s' ajouté dans la scène.", cube_id.c_str());
            }
        }

        // Démarrage de l'action server
        as_.start();
        ROS_INFO("PickPlace Action Server démarré.");
    }

    void executeCB(const moveit_planning::PickPlaceGoalConstPtr &goal) {
        // Ouverture pince
        feedback_.current_phase = "Ouverture pince";
        as_.publishFeedback(feedback_);
        manager_.openGripper();

        feedback_.current_phase = "Pick";
        as_.publishFeedback(feedback_);
        tf2::Vector3 n_local(1, 0, 0);  // axe X
        tf2::Vector3 o_local(0, 1, 0);  // axe Y
        if (!manager_.pick(goal->object_id,
                            loadObjectPose("/" + goal->object_id),
                            loadObjectSize("/" + goal->object_id + "/size"),
                             n_local, o_local, 0)) {
            result_.success = false;
            result_.message = "Échec du pick";
            as_.setAborted(result_);
            return;
        }

        feedback_.current_phase = "Place";
        as_.publishFeedback(feedback_);

        SolverType solver = loadSolver(goal->solver_name);

        if (!manager_.place(goal->object_id, goal->goal_pose.pose, goal->goal_pose.pose, solver)) {
            result_.success = false;
            result_.message = "Échec du place";
            as_.setAborted(result_);
            return;
        }

        result_.success = true;
        result_.message = "Pick & Place terminé avec succès";
        as_.setSucceeded(result_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place_action_server");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    PickPlaceActionServer server("pick_place");
    ros::waitForShutdown();
}