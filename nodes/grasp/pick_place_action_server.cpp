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
        as_.start();
        ROS_INFO("PickPlace Action Server démarré.");
    }

    void executeCB(const moveit_planning::PickPlaceGoalConstPtr &goal) {
        // Ouverture pince
        feedback_.current_step = "Ouverture pince";
        as_.publishFeedback(feedback_);
        manager_.openGripper();

        // Charger infos de l’objet
        auto obj_size = loadObjectSize("/" + goal->object_id + "/size");
        auto obj_pose = loadObjectPose("/" + goal->object_id);

        // Choix face de grasp
        feedback_.current_step = "Choix face grasp";
        as_.publishFeedback(feedback_);
        GraspChoice grasp;
        if (!check(chooseGraspFace(nh_, manager_.getGripperGroup(), obj_size, grasp), "choix face")) {
            result_.success = false;
            result_.message = "Echec choix grasp";
            as_.setAborted(result_);
            return;
        }

        // Approche
        feedback_.current_step = "Approche";
        as_.publishFeedback(feedback_);
        if (!check(manager_.approach(obj_pose, grasp.n_local, grasp.in_plane_axis), "approche")) {
            result_.success = false;
            result_.message = "Echec approche";
            as_.setAborted(result_);
            return;
        }

        // Grip
        feedback_.current_step = "Grip";
        as_.publishFeedback(feedback_);
        if (!check(manager_.grip(obj_pose, obj_size, grasp.n_local, grasp.in_plane_axis, grasp.face_index), "grip")) {
            result_.success = false;
            result_.message = "Echec grip";
            as_.setAborted(result_);
            return;
        }

        manager_.attachObject(goal->object_id);

        // Déplacement vers goal
        feedback_.current_step = "Déplacement";
        as_.publishFeedback(feedback_);
        SolverType solver = loadSolver(goal->solver_name.empty() ? "/goal_solver" : goal->solver_name);

        if (!check(manager_.moveToGoal(goal->goal_pose.pose, solver), "moveToGoal")) {
            result_.success = false;
            result_.message = "Echec déplacement";
            as_.setAborted(result_);
            return;
        }

        // Release
        feedback_.current_step = "Release";
        as_.publishFeedback(feedback_);
        manager_.openGripper();
        manager_.detachObject(goal->object_id);

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
