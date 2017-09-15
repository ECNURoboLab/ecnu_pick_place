//
// Created by sun on 17-9-15.
//

#include <ecnu_pick_place/ECNUPickPlace.h>
namespace {

    void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration,
                                      trajectory_msgs::JointTrajectory &grasp_pose) {
        grasp_pose.joint_names.reserve(target_values.size());
        grasp_pose.points.resize(1);
        grasp_pose.points[0].positions.reserve(target_values.size());

        for (std::map<std::string, double>::iterator it = target_values.begin();
             it != target_values.end(); ++it) {
            grasp_pose.joint_names.push_back(it->first);
            grasp_pose.points[0].positions.push_back(it->second);
        }
        grasp_pose.points[0].time_from_start = duration;
    }
}

namespace ecnu_pick_place{



    ECNUPickPlace::ECNUPickPlace(ros::NodeHandle &nh, const string &planner_id)
                                    :grasp_ac_("gpd_adapter_action", true)
                                    ,gripper_("gripper")
    {
        //! moveit 规划空间
        planning_scene_interface_.reset(new PlanningSceneInterface);
        planning_helper_.reset(new PlanningHelper("manipulator"));
        planning_helper_->setAllowedPlanningTime(5);
        planning_helper_->setPlanningAttempts(5);
        planning_helper_->setPlanningGroupName("manipulator");
        planning_helper_->setSupportSurfaceName("table");

        pub_attach_coll_obj_ = nh.advertise<AttachedCollisionObject>("attached_collision_object", 10);
        ROS_INFO("waiting for gpd adapter action to start ....");
        grasp_ac_.waitForServer();
        ROS_INFO("Action server got .....");
        start_pose_ = getStartPose();
        goal_pose_ = getGoalPose();
        //! 等待一会
        ros::Duration(2).sleep();
    }

    ECNUPickPlace::~ECNUPickPlace()
    {

    }

    bool ECNUPickPlace::start()
    {
        createEnvironment();
        bool found = false;
        while (!found && ros::ok()) {

            if (!pick(start_pose_, OBJ_ID)) {
                ROS_ERROR_STREAM_NAMED("pick_place", "Pick failed. Retrying.");
                cleanupACO(OBJ_ID);
            } else {
                ROS_INFO_STREAM_NAMED("pick_place",	"Done with pick!");
                found = true;
            }
        }

        ROS_INFO_STREAM_NAMED("simple_pick_place", "Waiting to put...");
        ros::Duration(5.5).sleep();

        bool placed = false;
        while (!placed && ros::ok()) {
            if (!place(goal_pose_, OBJ_ID)) {
                ROS_ERROR_STREAM_NAMED("pick_place", "Place failed.");
            } else {
                ROS_INFO_STREAM_NAMED("pick_place", "Done with place");
                placed = true;
            }
        }

        ROS_INFO_STREAM_NAMED("pick_place", "Pick and place cycle complete");

        ros::Duration(2).sleep();

        ROS_INFO("Finished");

        return true;

    }
    /**
     * 在环境中添加一个小圆柱，作为被抓取物体
     */

    void ECNUPickPlace::createEnvironment()
    {
        // remove the existing objects
        std::vector<std::string> object_ids;
        object_ids.push_back(OBJ_ID);
        planning_scene_interface_->removeCollisionObjects(object_ids);

        // create the objects
        std::vector<moveit_msgs::CollisionObject> collision_objects;

        // create the object
        moveit_msgs::CollisionObject object;
        object.header.frame_id = BASE_LINK;

        // the id of the object used to identify it.
        object.id = OBJ_ID;

        // create the object's sharp
        shape_msgs::SolidPrimitive cylinder;
        cylinder.type = cylinder.CYLINDER;
        cylinder.dimensions.resize(2);
        cylinder.dimensions[0] = CYCLINDER_HEIGHT;
        cylinder.dimensions[1] = CYCLINDER_RADIUS;

        geometry_msgs::Pose cylinder_pose = start_pose_;

        object.primitives.push_back(cylinder);
        object.primitive_poses.push_back(cylinder_pose);
        object.operation = object.ADD;


        collision_objects.push_back(object);
        ROS_INFO_STREAM("Add an object into the world ......");
        planning_scene_interface_->addCollisionObjects(collision_objects);
        ros::Duration(2).sleep();
    }

    geometry_msgs::Pose ECNUPickPlace::getStartPose()
    {
        filtered_cloud_publisher::GpdAdapterGoal goal;
        filtered_cloud_publisher::GpdAdapterResultConstPtr res;
        grasp_ac_.sendGoal(goal);
        grasp_ac_.waitForResult(ros::Duration(5));
        actionlib::SimpleClientGoalState state = grasp_ac_.getState();
        while (state != state.SUCCEEDED)
        {
            ROS_INFO("Waiting for grasp pose server");
            ros::Duration(0.5).sleep();
        }
        if(state == state.SUCCEEDED)
        {
            res = grasp_ac_.getResult();
            return res->pose;
        } else
        {
            ROS_INFO("Could not get the grasp pose!!!!!");
            exit(1);
        }
    }

    geometry_msgs::Pose ECNUPickPlace::getGoalPose()
    {
        geometry_msgs::Pose goal_pose;
        goal_pose.position.x = 0.4;
        goal_pose.position.y = 0.3;
        goal_pose.position.z = CYCLINDER_HEIGHT/2;

        goal_pose.orientation.w = 1;
        return goal_pose;
    }

    trajectory_msgs::JointTrajectory ECNUPickPlace::getPreGraspPosture()
    {
        /**this means the gripper open trajectory**/
        trajectory_msgs::JointTrajectory t;
        jointValuesToJointTrajectory(gripper_.getNamedTargetValues("open"), ros::Duration(1.0), t);
        return t;
    }

    trajectory_msgs::JointTrajectory ECNUPickPlace::getGraspPosture()
    {
        /**this means the gripper close trajectory**/
        trajectory_msgs::JointTrajectory t;
        jointValuesToJointTrajectory(gripper_.getNamedTargetValues("closed"), ros::Duration(1.0), t);
        return t;
    }

    void ECNUPickPlace::cleanupACO(const string &name)
    {
        // Clean up old attached collision object
        moveit_msgs::AttachedCollisionObject aco;
        aco.object.header.stamp = ros::Time::now();
        aco.object.header.frame_id = BASE_LINK;

        //aco.object.id = name;
        aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

        aco.link_name = "tool0";

        ros::WallDuration(0.1).sleep();
        pub_attach_coll_obj_.publish(aco);
    }

    bool ECNUPickPlace::pick(geometry_msgs::Pose &start_pose, const string &name)
    {
        ROS_WARN_STREAM("picking object "<< name);

        std::vector<Grasp> grasps;

        // Pick grasp
        generateGrasps(start_pose_, grasps);

        PlanningResultPtr plan = planning_helper_->plan_pick(name, grasps);
        if(plan->status == PlanningHelper::SUCCESS) {
            ROS_INFO("Planning pickup phase sucessfully completed!");
        } else {
            ROS_WARN("Planning pickup phase failed!");
            return false;
        }

        cout << ("Should the planned trajectory be executed (y/n)? ");
        string input;
        cin >> input;
        if(input == "y") {
            ROS_INFO("Executing pickup...");
            return planning_helper_->execute(plan);
        } else {
            ROS_WARN("Execution stopped by user!");
            return false;
        }
    }

    bool ECNUPickPlace::place(const geometry_msgs::Pose &goal_pose, string name)
    {
        ROS_WARN_STREAM_NAMED("pick_place", "Placing "<< name);

        std::vector<PlaceLocation> place_locations;

        trajectory_msgs::JointTrajectory post_place_posture = getPreGraspPosture();

        // Re-usable datastruct
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = BASE_LINK;
        // pose_stamped.header.stamp = ros::Time::now();

        // Create 360 degrees of place location rotated around a center
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 20) {
            pose_stamped.pose = goal_pose;

            // Orientation
            Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
            pose_stamped.pose.orientation.x = quat.x();
            pose_stamped.pose.orientation.y = quat.y();
            pose_stamped.pose.orientation.z = quat.z();
            pose_stamped.pose.orientation.w = quat.w();

            // Create new place location
            PlaceLocation place_loc;

            place_loc.place_pose = pose_stamped;

            // Approach
            GripperTranslation gripper_approach;
            // gripper_approach.direction.header.stamp = ros::Time::now();
            gripper_approach.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
            gripper_approach.min_distance = 0.1;
            gripper_approach.direction.header.frame_id = BASE_LINK;
            gripper_approach.direction.vector.x = 0;
            gripper_approach.direction.vector.y = 0;
            gripper_approach.direction.vector.z = -1;
            place_loc.pre_place_approach = gripper_approach;

            // Retreat
            GripperTranslation gripper_retreat;
            // gripper_retreat.direction.header.stamp = ros::Time::now();
            gripper_retreat.desired_distance = 0.2; // The distance the origin of a robot link needs to travel
            gripper_retreat.min_distance = 0.1;
            gripper_retreat.direction.header.frame_id = EE_LINK;
            gripper_retreat.direction.vector.x = 0;
            gripper_retreat.direction.vector.y = 0;
            gripper_retreat.direction.vector.z = -1; // Retreat direction (pos z axis)
            place_loc.post_place_retreat = gripper_retreat;

            // Post place posture - use same as pre-grasp posture (the OPEN command)
            place_loc.post_place_posture = post_place_posture;

            place_locations.push_back(place_loc);
        }

//		moveit_msgs::OrientationConstraint oc;
//		oc.header.frame_id = BASE_LINK;
//		oc.link_name = EE_PARENT_LINK;

//		oc.orientation.x = 0;
//		oc.orientation.y = 0;
//		oc.orientation.z = 0;
//		oc.orientation.w = 1;

//		oc.absolute_x_axis_tolerance = 0.3;
//		oc.absolute_y_axis_tolerance = 0.3;
//		oc.absolute_z_axis_tolerance = 0.3;

//		oc.weight = 1;

//		moveit_msgs::Constraints constraints;
//		constraints.orientation_constraints.push_back(oc);

        PlanningResultPtr plan = planning_helper_->plan_place(name, place_locations);

        if(plan->status == PlanningHelper::SUCCESS) {
            ROS_INFO("Planning placement phase sucessfully completed!");
        } else {
            ROS_WARN("Planning placement phase failed!");
            return false;
        }

        cout << ("Should the planned trajectory be executed (y/n)? ");
        string input;
        cin >> input;
        if(input == "y") {
            ROS_INFO("Executing placement...");
            return planning_helper_->execute(plan);
        } else {
            ROS_WARN("Execution stopped by user!");
            return false;
        }
    }

    bool ECNUPickPlace::generateGrasps(geometry_msgs::Pose &pose, vector<Grasp> &grasps)
    {

        // ---------------------------------------------------------------------------------------------
        // Create a transform from the object's frame to /base_link
        Eigen::Affine3d global_transform;
        tf::poseMsgToEigen(pose, global_transform);

        // ---------------------------------------------------------------------------------------------
        // Grasp parameters

        // -------------------------------
        // Create pre-grasp posture (Gripper open)
        trajectory_msgs::JointTrajectory pre_grasp_posture = getPreGraspPosture();

        // -------------------------------
        // Create grasp posture (Gripper closed)
        trajectory_msgs::JointTrajectory grasp_posture = getGraspPosture();

        // Create re-usable approach motion
        moveit_msgs::GripperTranslation gripper_approach;
        // gripper_approach.direction.header.stamp = ros::Time::now();
        gripper_approach.desired_distance = 0.13; // The distance the origin of a robot link needs to travel
        gripper_approach.min_distance = 0.1; // half of the desired? Untested.

        // Create re-usable retreat motion
        moveit_msgs::GripperTranslation gripper_retreat;
        // gripper_retreat.direction.header.stamp = ros::Time::now();
        gripper_retreat.desired_distance = 0.13; // The distance the origin of a robot link needs to travel
        gripper_retreat.min_distance = 0.1; // half of the desired? Untested.

        // Create re-usable blank pose
        geometry_msgs::PoseStamped grasp_pose_msg;
        // grasp_pose_msg.header.stamp = ros::Time::now();
        grasp_pose_msg.header.frame_id = BASE_LINK;
        Eigen::Affine3d tf_bottom_grasper;

        tf_bottom_grasper.setIdentity();
        tf_bottom_grasper.translation() = Eigen::Vector3d(0,0,-0.13);
        tf::poseEigenToMsg(global_transform*tf_bottom_grasper,grasp_pose_msg.pose);

        moveit_msgs::Grasp res_grasp;
        res_grasp.id = "Grasp0";
        res_grasp.pre_grasp_posture = pre_grasp_posture;
        res_grasp.grasp_posture = grasp_posture;
        res_grasp.grasp_pose = grasp_pose_msg;
        res_grasp.grasp_quality = 1;
        res_grasp.allowed_touch_objects.push_back(OBJ_ID);
        res_grasp.max_contact_force = 0;
        // Approach
        gripper_approach.direction.header.frame_id = EE_LINK;
        gripper_approach.direction.vector.x = 0;
        gripper_approach.direction.vector.y = 0;
        gripper_approach.direction.vector.z = 1;
        res_grasp.pre_grasp_approach = gripper_approach;

        // Retreat
        gripper_retreat.direction.header.frame_id = BASE_LINK;
        gripper_retreat.direction.vector.x = 0;
        gripper_retreat.direction.vector.y = 0;
        gripper_retreat.direction.vector.z = 1;
        res_grasp.post_grasp_retreat = gripper_retreat;

        grasps.push_back(res_grasp);


        ROS_INFO_STREAM_NAMED("pick_place", "Generated " << grasps.size() << " grasps.");

        return true;
    }

}
