//
// Created by sun on 17-9-15.
//

#ifndef ECNU_PICK_PLACE_ECNUPICKPLACE_H
#define ECNU_PICK_PLACE_ECNUPICKPLACE_H
//ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
//MOVEIT
#include <ecnu_pick_place/PlanningHelper.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/Grasp.h>
//TF EIGEN
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
//BOOST
#include <boost/shared_ptr.hpp>
//GraspAdapter
#include <filtered_cloud_publisher/GpdAdapterAction.h>

using namespace std;
using namespace moveit::planning_interface;
using namespace moveit_msgs;
using namespace uibk_planning_node;

namespace ecnu_pick_place{
    static const std::string BASE_LINK = "base_link";
    static const std::string EE_LINK = "tool0";
    static const std::string OBJ_ID = "cylinder1";

    static const std::string SUPPORT_SUFACE_NAME = "base_link";
    static const double SUPPORT_SURFACE_HIGHT = 0.08;

    static const double CYCLINDER_HEIGHT = 0.02;
    static const double CYCLINDER_RADIUS = 0.02;


    class ECNUPickPlace {
    public:


        ECNUPickPlace(ros::NodeHandle& nh, const string& planner_id);
        ~ECNUPickPlace();
        bool start();
        void createEnvironment();
        trajectory_msgs::JointTrajectory getPreGraspPosture();
        trajectory_msgs::JointTrajectory getGraspPosture();
        geometry_msgs::Pose getStartPose();
        geometry_msgs::Pose getGoalPose();
        void cleanupACO(const string &name);
        bool pick(geometry_msgs::Pose& start_pose, const string &name);
        bool place(const geometry_msgs::Pose& goal_pose, string name);
        bool generateGrasps(geometry_msgs::Pose& pose, vector<Grasp>& grasps);


    private:
        boost::shared_ptr<PlanningSceneInterface> planning_scene_interface_;
        PlanningHelperPtr planning_helper_;
        moveit::planning_interface::MoveGroup gripper_;
        geometry_msgs::Pose start_pose_;
        geometry_msgs::Pose goal_pose_;

        ros::Publisher pub_attach_coll_obj_;

        actionlib::SimpleActionClient<filtered_cloud_publisher::GpdAdapterAction> grasp_ac_;

    };
}




#endif //ECNU_PICK_PLACE_ECNUPICKPLACE_H
