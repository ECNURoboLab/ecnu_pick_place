/*
 * plan_execution.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Griesser
 */

#include <ecnu_pick_place/PlanningHelper.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std;

namespace uibk_planning_node {

PlanningHelper::PlanningHelper(const string &name) {

	pub_ = node_handle_.advertise<moveit_msgs::PickupGoal>("pickup_message", 1, true);
	/* Load the robot model */
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	/* Get a shared pointer to the model */
	// instance of our robot model loaded from URDF
	ROS_INFO("Loading robot model from URDF");
	robot_model_ = robot_model_loader.getModel();
	if(!robot_model_) {
		ROS_FATAL_STREAM("Unable to construct robot model!");
		throw runtime_error("Unable to construct robot model!");
	}
	if (!robot_model_->hasJointModelGroup(name)) {
		std::string error = "Group '" + name + "' was not found.";
		ROS_FATAL_STREAM(error);
		throw std::runtime_error(error);
	}
	joint_model_group_ = robot_model_->getJointModelGroup(name);

	// try to find the name of the end effector...
	const vector<const robot_model::JointModelGroup *> eefs = robot_model_->getEndEffectors();
	for(size_t i = 0; i < eefs.size(); ++i) {
		string eefName = eefs[i]->getName();
		string parent_name = eefs[i]->getEndEffectorParentGroup().first;
		if(parent_name == name) { end_effector_ = eefName;
			break;
		}
	}
	// get the name of the end effector link in the arm...
	end_effector_link_ = joint_model_group_->getLinkModelNames().back();
	ROS_INFO("End effector link: %s", end_effector_link_.c_str());
	group_name_ = name;
	planning_time_ = 5.0;
	planning_attempts_ = 5;
	goal_joint_tolerance_ = 1e-4;
	goal_position_tolerance_ = 1e-4; // 0.1 mm
	goal_orientation_tolerance_ = 1e-3; // ~0.1 deg
	planner_id_ = "";

	ROS_INFO("Connecting to pickup client...");
	string pickup_topic = "pickup";
	pick_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(pickup_topic, true));
	pick_action_client_->waitForServer();

	ROS_INFO("Connecting to place client...");
	string place_topic = "place";
	place_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(place_topic, true));
	place_action_client_->waitForServer();

	ROS_INFO("Connecting to move_group client...");
	string move_group_topic = "move_group";
	move_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(move_group_topic, true));
	move_action_client_->waitForServer();

	execution_client_ = node_handle_.serviceClient<ExecuteKnownTrajectory>("execute_kinematic_path");
	attached_object_publisher_ = node_handle_.advertise<AttachedCollisionObject>("attached_collision_object", 1, false);

}

PlanningResultPtr PlanningHelper::plan_pick(const string &object_id, const std::vector<Grasp> &grasps) {

	PlanningResultPtr result;
	result.reset(new PlanningResult);
	result->type = PlanningHelper::PICK;

	if (!pick_action_client_) {
		ROS_ERROR("Pick action client not found");

		result->status = PlanningHelper::FAILURE;
		result->status_msg = "Pick action client not found";
		return result;
	}

	if (!pick_action_client_->isServerConnected()) {
		ROS_ERROR("Pick action server not connected");

		result->status = PlanningHelper::FAILURE;
		result->status_msg = "Pick action server not connected";
		return result;
	}

	moveit_msgs::PickupGoal goal;

	goal.target_name = object_id;
	goal.group_name = group_name_;
	goal.end_effector = end_effector_;
	goal.allowed_planning_time = planning_time_;
	goal.support_surface_name = support_surface_;
	goal.planner_id = planner_id_;

	if (!support_surface_.empty()) {
		goal.allow_gripper_support_collision = true;
	}

	ROS_INFO("Pickup goal constructed for group '%s' and end effector '%s'", goal.group_name.c_str(), goal.end_effector.c_str());

	goal.possible_grasps = grasps;
	goal.planning_options.look_around = CAN_LOOK;
	goal.planning_options.replan = ALLOW_REPLAN;
	goal.planning_options.replan_delay = 2.0;
	goal.planning_options.plan_only = true;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	ROS_INFO("Publishing pickup message");
	pub_.publish(goal);

	pick_action_client_->sendGoal(goal);

	if (!pick_action_client_->waitForResult()) {
		ROS_INFO_STREAM("Pickup action returned early");
	}

    moveit_msgs::PickupResultConstPtr res = pick_action_client_->getResult();

	if (pick_action_client_->getState()	== actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Call to pick action server succeeded!");

		result->status = PlanningHelper::SUCCESS;
		result->status_msg = "Call to pick action server succeeded!";
		result->object_id = object_id;
		result->start_state = res->trajectory_start;
		result->trajectory_descriptions = res->trajectory_descriptions;
		result->trajectory_stages = res->trajectory_stages;

	} else {
		ROS_WARN_STREAM("Fail: " << pick_action_client_->getState().toString() << ": " << pick_action_client_->getState().getText());

		result->status = PlanningHelper::FAILURE;
		stringstream ss;
		ss << "Planning failed with status code '" << res->error_code.val << "'";
		result->status_msg = ss.str();
	}

    return result;
}

PlanningResultPtr PlanningHelper::plan_place(const string &object_id, const vector<PlaceLocation> &locations) {

	PlanningResultPtr result;
	result.reset(new PlanningResult);
	result->type = PlanningHelper::PLACE;

	if (!place_action_client_) {
		ROS_ERROR_STREAM("Place action client not found");

		result->status = PlanningHelper::FAILURE;
		result->status_msg = "Place action client not found";
		return result;
	}
	if (!place_action_client_->isServerConnected()) {
		ROS_ERROR_STREAM("Place action server not connected");

		result->status = PlanningHelper::FAILURE;
		result->status_msg = "Place action server not connected";
		return result;
	}

	moveit_msgs::PlaceGoal goal;

	goal.attached_object_name = object_id;
	goal.group_name = group_name_;
	goal.allowed_planning_time = planning_time_;
	goal.support_surface_name = support_surface_;
	goal.planner_id = "RRTConnectkConfigDefault";

	if (!support_surface_.empty()) {
		goal.allow_gripper_support_collision = true;
	}

	goal.place_locations = locations;
	goal.planning_options.plan_only = true;
	goal.planning_options.look_around = CAN_LOOK;
	goal.planning_options.replan = ALLOW_REPLAN;
	goal.planning_options.replan_delay = 2;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	place_action_client_->sendGoal(goal);

	ROS_DEBUG("Sent place goal with %d locations", (int ) goal.place_locations.size());

	if (!place_action_client_->waitForResult()) {
		ROS_INFO_STREAM("Place action returned early");
	}

	moveit_msgs::PlaceResultConstPtr res = place_action_client_->getResult();

	if (place_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Call to place action server succeeded!");

		result->status = PlanningHelper::SUCCESS;
		result->status_msg = "Call to pick action server succeeded!";
		result->object_id = object_id;
		result->start_state = res->trajectory_start;
		result->trajectory_descriptions = res->trajectory_descriptions;
		result->trajectory_stages = res->trajectory_stages;

	} else {
		ROS_WARN_STREAM("Fail: " << place_action_client_->getState().toString() << ": " << place_action_client_->getState().getText());

		result->status = PlanningHelper::FAILURE;
		stringstream ss;
		ss << "Planning failed with status code '" << res->error_code.val << "'";
		result->status_msg = ss.str();
	}

	return result;
}

PlanningResultPtr PlanningHelper::plan(const geometry_msgs::Pose &pose_goal) {

	PlanningResultPtr result;
	result.reset(new PlanningResult);
	result->type = PlanningHelper::GOAL;

	if (!move_action_client_) {
		ROS_ERROR_STREAM("Place action client not found");

		result->status = PlanningHelper::FAILURE;
		result->status_msg = "Move action client not found";
		return result;
	}
	if (!move_action_client_->isServerConnected()) {
		ROS_ERROR_STREAM("Place action server not connected");

		result->status = PlanningHelper::FAILURE;
		result->status_msg = "Move action server not connected";
		return result;
	}

	moveit_msgs::MoveGroupGoal goal;
	goal.request.group_name = group_name_;
	goal.request.num_planning_attempts = 5;
	goal.request.allowed_planning_time = planning_time_;
	goal.request.planner_id = planner_id_;
//	goal.request.workspace_parameters = workspace_parameters_;

	geometry_msgs::PoseStamped p;
	p.header.frame_id = FRAME_ID;
	p.header.stamp = ros::Time::now();
	p.pose = pose_goal;

	vector<Constraints> constraints;
	Constraints c =	kinematic_constraints::constructGoalConstraints(end_effector_link_,	p,
																	goal_position_tolerance_,
																	goal_orientation_tolerance_);

	goal.planning_options.plan_only = true;
	goal.planning_options.look_around = false;
	goal.planning_options.replan = false;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
	goal.request.goal_constraints.push_back(c);

	move_action_client_->sendGoal(goal);

	ROS_DEBUG("Sent planning request for pose goal");

	if (!move_action_client_->waitForResult()) {
		ROS_INFO_STREAM("MoveGroup action returned early");
	}

	moveit_msgs::MoveGroupResultConstPtr res = move_action_client_->getResult();

	if (move_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Call to move_group action server succeeded!");

		result->status = PlanningHelper::SUCCESS;
		result->status_msg = "Call to pick action server succeeded!";
		result->start_state = res->trajectory_start;
		result->trajectory_descriptions.push_back("target");
		result->trajectory_stages.push_back(res->planned_trajectory);

	} else {
		ROS_WARN_STREAM("Fail: " << move_action_client_->getState().toString() << ": " << move_action_client_->getState().getText());

		result->status = PlanningHelper::FAILURE;
		stringstream ss;
		ss << "Planning failed with status code '" << res->error_code.val << "'";
		result->status_msg = ss.str();
	}

	return result;
}

bool PlanningHelper::execute(const PlanningResultPtr& plan) {
	for (size_t i = 0; i < plan->trajectory_stages.size(); ++i) {
		string &description = plan->trajectory_descriptions[i];

		ROS_INFO_STREAM("Executing stage '" << description << "'...");
		if(!execute(plan->trajectory_stages[i])) {
			return false;
		}

		if(description == "grasp" && plan->type == PlanningHelper::PICK) {
			attachObject(plan->object_id);
		}
		else if(description == "grasp" && plan->type == PlanningHelper::PLACE) {
			detachObject(plan->object_id);
		}
	}

	return true;

}

bool PlanningHelper::execute(const RobotTrajectory &trajectory) {
	ExecuteKnownTrajectory msg;
	ExecuteKnownTrajectoryRequest &request = msg.request;

	request.wait_for_execution = true;
	request.trajectory = trajectory;

	bool success = execution_client_.call(msg);

	if (success) {
		MoveItErrorCodes &code = msg.response.error_code;

		if (code.val == MoveItErrorCodes::SUCCESS) {
			ROS_INFO("Execution finished successfully.");
		} else {
			ROS_ERROR("Execution finished with error_code '%d'", code.val);
			return false;
		}

	} else {
		ROS_ERROR("Execution failed!");
		return false;
	}

	return true;
}

bool PlanningHelper::attachObject(const string &object) {
	moveit_msgs::AttachedCollisionObject aco;

	aco.object.id = object;
	aco.link_name = end_effector_link_;
	aco.touch_links = robot_model_->getEndEffector(end_effector_)->getLinkModelNames();
	aco.object.operation = moveit_msgs::CollisionObject::ADD;
	attached_object_publisher_.publish(aco);

	return true;
}

void PlanningHelper::setPlanningAttempts(int value) {
	planning_attempts_ = value;
}

int PlanningHelper::getPlanningAttempts() {
	return planning_attempts_;
}

bool PlanningHelper::detachObject(const string &object) {
	moveit_msgs::AttachedCollisionObject aco;

	aco.object.id = object;
	aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
	aco.link_name = end_effector_link_;

	attached_object_publisher_.publish(aco);

	return true;
}

void PlanningHelper::setPlanningGroupName(const string& name) {
	if (!robot_model_->hasJointModelGroup(name)) {
		std::string error =	"No JointModel group with given name exists!";
		ROS_FATAL_STREAM(error);
		throw std::runtime_error(error);
	}
	group_name_ = name;
}

string PlanningHelper::getPlanningGroupName() {
	return group_name_;
}

void PlanningHelper::setPlannerId(const string& planner_id) {
	planner_id_ = planner_id;
}

string PlanningHelper::getPlannerId() {
	return planner_id_;
}

void PlanningHelper::setAllowedPlanningTime(double value) {
	planning_time_ = value;
}

double PlanningHelper::getAllowedPlanningTime() {
	return planning_time_;
}

void PlanningHelper::setSupportSurfaceName(const string name) {
	support_surface_ = name;
}

string PlanningHelper::getSupportSurfaceName() {
	return support_surface_;
}

} // end namespace
