#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_cartesian_plan_plugin/generate_cartesian_path.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


GenerateCartesianPath::GenerateCartesianPath(QObject *parent)
{
    //to add initializations
    init();

}
GenerateCartesianPath::~GenerateCartesianPath()
{
  /*! The destructor resets the moveit_group_ and the kinematic_state of the robot.
  */
  moveit_group_.reset();
  kinematic_state_.reset();
  robot_model_loader.reset();
  kmodel_.reset();
  delete tfListener;
}

void GenerateCartesianPath::init()
{
  /*! Initialize the MoveIt parameters:
        - MoveIt group
        - Kinematic State is the current kinematic congiguration of the Robot
        - Robot model which handles getting the Robot Model
        - Joint Model group which are necessary for checking if Way-Point is outside the IK Solution
        .
  */
  selected_plan_group = 0;
  robot_model_loader = RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
  kmodel_ = robot_model_loader->getModel();

  end_eff_joint_groups = kmodel_->getEndEffectors();


  ROS_INFO_STREAM("size of the end effectors is: "<<end_eff_joint_groups.size());
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  if (end_eff_joint_groups.empty())
  {
    std::vector< std::string > group_names_tmp_;
    const moveit::core::JointModelGroup *  end_eff_joint_groups_tmp_;
    group_names_tmp_ = kmodel_->getJointModelGroupNames();

    for(int i=0;i<group_names_tmp_.size();i++)
    {

    end_eff_joint_groups_tmp_ = kmodel_->getJointModelGroup(group_names_tmp_.at(i));
    if(end_eff_joint_groups_tmp_->isChain())
    {
      group_names.push_back(group_names_tmp_.at(i));
    }
    else
    {
      ROS_WARN_STREAM("The group:" << end_eff_joint_groups_tmp_->getName() <<" is not a Chain. Depreciate it!!");
    }
    }
  }
  else
  {
    for(int i=0;i<end_eff_joint_groups.size();i++)
    { 
      if(end_eff_joint_groups.at(i)->isChain())
      {
      const std::string& parent_group_name = end_eff_joint_groups.at(i)->getName();
      group_names.push_back(parent_group_name);

      ROS_INFO_STREAM("Group name:"<< group_names.at(i));
      }
      else
      {
          ROS_INFO_STREAM("This group is not a chain. Find the parent of the group");
          const std::pair< std::string, std::string > & parent_group_name = end_eff_joint_groups.at(i)->getEndEffectorParentGroup();
          group_names.push_back(parent_group_name.first);
      }
    }
  }

  ROS_INFO_STREAM("Group name:"<< group_names[selected_plan_group]);

  moveit_group_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(group_names[selected_plan_group]));
  kinematic_state_ = moveit::core::RobotStatePtr(new robot_state::RobotState(kmodel_));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kmodel_->getJointModelGroup(group_names[selected_plan_group]);

  // Cartesian planning and execution
  cart_plan_action_client = boost::shared_ptr<actionlib::SimpleActionClient<peanut_descartes::GetCartesianPathAction>>(new actionlib::SimpleActionClient<peanut_descartes::GetCartesianPathAction>("/compute_cartesian_path", true));
  cart_plan_action_client->waitForServer(ros::Duration(2.0));

  cart_exec_action_client = boost::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>>(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>("/execute_trajectory", true));
  cart_exec_action_client->waitForServer(ros::Duration(2.0));
  
}

void GenerateCartesianPath::setCartParams(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_, std::string robot_model_frame_, bool fix_start_state_)
{
  /*! Set the necessary parameters for the MoveIt and the Cartesian Path Planning.
      These parameters correspond to the ones that the user has entered or the default ones before the execution of the Cartesian Path Planner.
  */
  ROS_INFO_STREAM("MoveIt and Cartesian Path parameters from UI:\n MoveIt Plan Time:"<<plan_time_
                  <<"\n Cartesian Path Step Size:"<<cart_step_size_
                  <<"\n Jump Threshold:"<<cart_jump_thresh_
                  <<"\n Replanning:"<<moveit_replan_
                  <<"\n Avoid Collisions:"<<avoid_collisions_
                  <<"\n Robot model frame: "<<robot_model_frame_
                  <<"\n Start state is fixed: "<<fix_start_state_);

  PLAN_TIME_        = plan_time_;
  MOVEIT_REPLAN_    = moveit_replan_;
  CART_STEP_SIZE_   = cart_step_size_;
  CART_JUMP_THRESH_ = cart_jump_thresh_;
  AVOID_COLLISIONS_ = avoid_collisions_;
  ROBOT_MODEL_FRAME_ = robot_model_frame_;
  FIX_START_STATE_ = fix_start_state_;
}

void GenerateCartesianPath::moveToConfig(std::vector<double> config, bool plan_only) 
{
    Q_EMIT cartesianPathExecuteStarted();

    ROS_INFO("starting plan to config");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_group_->setPlanningTime(PLAN_TIME_);
    moveit_group_->allowReplanning (MOVEIT_REPLAN_);

    moveit_group_->setJointValueTarget(config);

    bool success = (moveit_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    
    if (!plan_only && success){
      moveit_group_->execute(my_plan);
    }

    Q_EMIT cartesianPathExecuteFinished();

}

void GenerateCartesianPath::moveToPose(std::vector<geometry_msgs::Pose> waypoints, const bool plan_only)
{
    Q_EMIT cartesianPathExecuteStarted();
    moveit::planning_interface::MoveItErrorCode freespace_error_code;

    moveit_group_->setPlanningTime(PLAN_TIME_);
    moveit_group_->allowReplanning (MOVEIT_REPLAN_);
    moveit_group_->setPoseReferenceFrame("base_link");
    robot_state::RobotStatePtr start_state = moveit_group_->getCurrentState(10.0);
    ROS_INFO_STREAM("the start state of the robot is " << start_state);
    moveit_group_->setStartStateToCurrentState();

    // If the start state is not fixed and waypoints size is 1, freespace plan and end
    if (waypoints.size() == 1 && !FIX_START_STATE_){ 
      ROS_INFO("Path contains just 1 waypoint.");
      ROS_INFO("Freespace planning to waypoint ...");
      moveit_group_->setStartState(*start_state);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      moveit_group_->setPlanningTime(PLAN_TIME_);
      moveit_group_->allowReplanning (MOVEIT_REPLAN_);
      moveit_group_->setPoseTarget(waypoints.at(0), "end_effector_link");
      bool success = (moveit_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success){
        freespace_error_code = moveit_group_->execute(my_plan);
        Q_EMIT cartesianPathCompleted(69);
        ROS_INFO_STREAM("Computed freespace path.");
        Q_EMIT cartesianPathExecuteFinished();
        return;
      }
      else {
        ROS_ERROR_STREAM("Could not compute freespace path.");
        Q_EMIT cartesianPathCompleted(-100);
        Q_EMIT cartesianPathExecuteFinished();
        return;
      }
    }
    
    ROS_INFO_STREAM("The frame planning occurs in is base_link the frame we are currently in is " << ROBOT_MODEL_FRAME_ << " transforming");
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link", ROBOT_MODEL_FRAME_ , ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      Q_EMIT cartesianPathExecuteFinished();
    }
    std::vector<geometry_msgs::Pose> waypoints_pose_copy;

    const geometry_msgs::Transform constTransform = transformStamped.transform;
    ROS_INFO_STREAM("transform used to transform into planning frame of base_link: " << transformStamped);
    tf::Transform transform_old_new;
    tf::transformMsgToTF(constTransform, transform_old_new);

    moveit_msgs::Constraints path_constraints;
    moveit_msgs::OrientationConstraint orientation_constraint;
    orientation_constraint.absolute_x_axis_tolerance = 0.0;
    orientation_constraint.weight = 0.0;

    moveit_msgs::PositionConstraint position_constraint;
    position_constraint.target_point_offset.x = 0.0;
    position_constraint.weight = 0.0;
    
    for (int i=0; i<waypoints.size(); i++)
    {
      tf::Transform waypoint_tf;
      geometry_msgs::Pose waypoint_pose;
      tf::poseMsgToTF (waypoints[i], waypoint_tf);
      waypoint_tf = transform_old_new*waypoint_tf;
      tf::poseTFToMsg (waypoint_tf, waypoint_pose);
      waypoints_pose_copy.push_back(waypoint_pose);
      path_constraints.position_constraints.push_back(position_constraint);
      path_constraints.orientation_constraints.push_back(orientation_constraint);
    }

    // If the start state is fixed then append the current pose to the list of poses for cartesian planning
    if (FIX_START_STATE_){ 
      geometry_msgs::TransformStamped eef_pos;
      try{
        eef_pos = tfBuffer.lookupTransform("base_link", "end_effector_link" , ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        Q_EMIT cartesianPathExecuteFinished();
        return;
      }
      geometry_msgs::Pose waypoint_pose;
      tf::Transform eef_pos_tf;
      tf::transformMsgToTF(eef_pos.transform, eef_pos_tf);
      tf::poseTFToMsg (eef_pos_tf, waypoint_pose);

      std::vector<geometry_msgs::Pose>::iterator it = waypoints_pose_copy.begin();
      waypoints_pose_copy.insert(it, waypoint_pose);
      moveit_group_->setStartState(*start_state);
      path_constraints.position_constraints.push_back(position_constraint);
      path_constraints.orientation_constraints.push_back(orientation_constraint);
    }

    // // //
    // 1. compute cartesian path
    // // //
    ROS_INFO_STREAM("Frame which poses are represented in " << ROBOT_MODEL_FRAME_);

    peanut_descartes::GetCartesianPathGoal cart_plan_goal;
    moveit_msgs::RobotState robot_state_msg;

    cart_plan_goal.header.frame_id = "base_link";
    cart_plan_goal.header.stamp = ros::Time::now();
    cart_plan_goal.group_name = "arm";

    cart_plan_goal.waypoints = waypoints_pose_copy;
    cart_plan_goal.max_step = CART_STEP_SIZE_;
    cart_plan_goal.jump_threshold = CART_JUMP_THRESH_;
    cart_plan_goal.avoid_collisions = AVOID_COLLISIONS_;
    cart_plan_goal.path_constraints = path_constraints;
    if(FIX_START_STATE_){
      std::vector<double> arm_states;
      start_state->copyJointGroupPositions("arm", arm_states);
      robot_state_msg.joint_state.position = arm_states;
      robot_state_msg.joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
      cart_plan_goal.start_state = robot_state_msg;
    }

    cart_plan_action_client->sendGoal(cart_plan_goal);
    cart_plan_action_client->waitForResult();
    peanut_descartes::GetCartesianPathResultConstPtr cart_plan_result = cart_plan_action_client->getResult();

    // Finally plan and execute the trajectory
    if(cart_plan_result->error_code.val != 1){
      ROS_ERROR("cartesian planning was not successful, returning");
      Q_EMIT cartesianPathCompleted(0);
      Q_EMIT cartesianPathExecuteFinished();
      return;
    }
    else{
      Q_EMIT cartesianPathCompleted(100);
    }

    if (plan_only){
      Q_EMIT cartesianPathExecuteFinished();
      return;
    }

    // // //
    // 2. compute and execute freespace plan
    // // //
    if (!FIX_START_STATE_){
      ROS_INFO_STREAM("Start state is not fixed. Plan and exec freespace plan");
      moveit::planning_interface::MoveGroupInterface::Plan freespace_plan;
      moveit_group_->setPlanningTime(PLAN_TIME_);
      moveit_group_->allowReplanning (MOVEIT_REPLAN_);
      std::vector<double> start_config = cart_plan_result->solution.joint_trajectory.points.front().positions;
      moveit_group_->setJointValueTarget(start_config);

      bool success = (moveit_group_->plan(freespace_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
      
      if (success){
        freespace_error_code = moveit_group_->execute(freespace_plan);
      }
      else {
        ROS_ERROR_STREAM("Could not compute freespace path to starting config of cartesian path");
        Q_EMIT cartesianPathExecuteFinished();
        return;
      }
    }

    // // //
    // 3. execute cartesian path
    // // //

    // Update the starting point to be the correct time
    if (freespace_error_code==freespace_error_code.SUCCESS || FIX_START_STATE_){
      moveit_msgs::ExecuteTrajectoryGoal cart_exec_goal;
      cart_exec_goal.trajectory.joint_trajectory = cart_plan_result->solution.joint_trajectory;
      cart_exec_goal.trajectory.joint_trajectory.header.stamp = ros::Time::now();
      cart_exec_action_client->sendGoal(cart_exec_goal);
      cart_exec_action_client->waitForResult();
      moveit_msgs::ExecuteTrajectoryResultConstPtr cart_exec_result = cart_exec_action_client->getResult();
    }

    Q_EMIT cartesianPathExecuteFinished();
}

void GenerateCartesianPath::cartesianPathHandler(std::vector<geometry_msgs::Pose> waypoints, const bool plan_only)
{
  /*! Since the execution of the Cartesian path is time consuming and can lead to locking up of the Plugin and the RViz enviroment the function for executing the Cartesian Path Plan has been placed in a separtate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting concurrent process for Cartesian Path");
  QFuture<void> future = QtConcurrent::run(this, &GenerateCartesianPath::moveToPose, waypoints, plan_only);
}

void GenerateCartesianPath::freespacePathHandler(std::vector<double> config, bool plan_only)
{
  /*! Since the execution of the Cartesian path is time consuming and can lead to locking up of the Plugin and the RViz enviroment the function for executing the Cartesian Path Plan has been placed in a separtate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting concurrent process for Freespace Planning");
  QFuture<void> future = QtConcurrent::run(this, &GenerateCartesianPath::moveToConfig, config, plan_only);
}

void GenerateCartesianPath::interpolate(Eigen::Affine3d start_pos, Eigen::Affine3d end_pos, float step_size, std::vector<Eigen::Affine3d>& interpolated)
{
  // calculate distance required to rotate
  Eigen::Quaterniond start_pos_rot{start_pos.rotation()};
  Eigen::Quaterniond end_pos_rot{end_pos.rotation()};
  double angDist = start_pos_rot.angularDistance(end_pos_rot);
  // calculate distance required to translate
  Eigen::Vector3d trans_diff = end_pos.translation() - start_pos.translation();
  double linDist = trans_diff.squaredNorm();
  // calculate nb intermediate end_poss based on max nb steps for rotation / translation
  unsigned int nbAngSteps = ceil(angDist / step_size);
  unsigned int nbLinSteps = ceil(linDist / step_size);
  unsigned int nbSteps = std::max(nbAngSteps, nbLinSteps);
  // find intermediate end_poss and add to list
  for (std::size_t step = 1; step < nbSteps; step++)
  {
    Eigen::Affine3d iend_pos;
    // translational interpolation
    Eigen::Vector3d ipos = start_pos.translation() + float(step) / nbSteps * trans_diff;
    // rotational interpolation
    Eigen::Affine3d irot = Eigen::Affine3d(start_pos_rot.slerp(float(step) / nbSteps, end_pos_rot));
    iend_pos = Eigen::Translation3d(ipos) * irot;
    interpolated.push_back(iend_pos);
  }
}

void GenerateCartesianPath::checkWayPointValidity(const std::vector<tf::Transform> waypoints, const int point_number)
{
  /*! This function is called every time the user updates the pose of the Way-Point and checks if the Way-Point is within the valid IK solution for the Robot.
      In the case when a point is outside the valid IK solution this function send a signal to the RViz enviroment to update the color of the Way-Point.
  */
  if (!check_ik){
   return;
  }
  try{
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link", ROBOT_MODEL_FRAME_ , ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      return;
    }
    const geometry_msgs::Transform constTransform = transformStamped.transform;
    tf::Transform transform_old_new;
    tf::transformMsgToTF(constTransform, transform_old_new);

    tf::Transform waypoint_tf;
    Eigen::Affine3d waypoint1;
    Eigen::Affine3d waypoint2;
    std::vector<Eigen::Affine3d> interpolated;
    const int p_idx = point_number - 1;

    // More than 1 point exists
    if (waypoints.size() > 1){
      // First point
      if (p_idx == 0){
        waypoint_tf = waypoints.at(p_idx);
        waypoint_tf = transform_old_new*waypoint_tf;
        tf::transformTFToEigen(waypoint_tf, waypoint1);

        waypoint_tf = waypoints.at(p_idx+1);
        waypoint_tf = transform_old_new*waypoint_tf;
        tf::transformTFToEigen(waypoint_tf, waypoint2);
        interpolate(waypoint1, waypoint2, CART_STEP_SIZE_, interpolated);
      }

      // Middle point
      if ((p_idx >= 1) && (p_idx < waypoints.size() - 1)){
        for (int i = p_idx-1; i <= p_idx; i++){
          waypoint_tf = waypoints.at(i);
          waypoint_tf = transform_old_new*waypoint_tf;
          tf::transformTFToEigen(waypoint_tf, waypoint1);

          waypoint_tf = waypoints.at(i+1);
          waypoint_tf = transform_old_new*waypoint_tf;
          tf::transformTFToEigen(waypoint_tf, waypoint2);
          interpolate(waypoint1, waypoint2, CART_STEP_SIZE_, interpolated);
        }
      }

      // Last point
      if (p_idx == waypoints.size()-1){
        waypoint_tf = waypoints.at(p_idx-1);
        waypoint_tf = transform_old_new*waypoint_tf;
        tf::transformTFToEigen(waypoint_tf, waypoint1);

        waypoint_tf = waypoints.at(p_idx);
        waypoint_tf = transform_old_new*waypoint_tf;
        tf::transformTFToEigen(waypoint_tf, waypoint2);
        interpolate(waypoint1, waypoint2, CART_STEP_SIZE_, interpolated);
      }
    }

    waypoint_tf = waypoints.at(p_idx);
    waypoint_tf = transform_old_new*waypoint_tf;
    tf::transformTFToEigen(waypoint_tf, waypoint1);
    interpolated.push_back(waypoint1);

    ROS_DEBUG_STREAM("Length of interpolated points: " << std::to_string(interpolated.size()));

    bool found_ik;
    int any_invalid = 0;
    geometry_msgs::Pose pointOutOfBounds;
    std::vector<geometry_msgs::Pose> out_of_bounds_interpolated;
    for (const auto& point : interpolated){
      found_ik = jaco3_kinematics::ik_exists(point, 125);
      if (!found_ik){
        any_invalid = 1;
        tf::poseEigenToTF(point, waypoint_tf);
        tf::poseTFToMsg(waypoint_tf, pointOutOfBounds);
        out_of_bounds_interpolated.push_back(pointOutOfBounds);
      }
    }
    Q_EMIT wayPointOutOfIK(point_number, any_invalid, out_of_bounds_interpolated);
  }
  catch(const std::exception &exc){
    ROS_ERROR_STREAM("Error is "<< exc.what());
  }
  catch(...){
    ROS_ERROR("An unknown error occured in check waypoint validity");
  }
}

void GenerateCartesianPath::initRvizDone()
{
  /*! Once the initialization of the RViz is has finished, this function sends the pose of the robot end-effector and the name of the base frame to the RViz enviroment.
      The RViz enviroment sets the User Interactive Marker pose and Add New Way-Point RQT Layout default values based on the end-effector starting position.
      The transformation frame of the InteractiveMarker is set based on the robot PoseReferenceFrame.
  */
  ROS_INFO("RViz is done now we need to emit the signal");

  if(moveit_group_->getEndEffectorLink().empty())
  {
    ROS_INFO("End effector link is empty");
    const std::vector< std::string > &  joint_names = joint_model_group_->getLinkModelNames();
    for(int i=0;i<joint_names.size();i++)
    {
      ROS_INFO_STREAM("Link " << i << " name: "<< joint_names.at(i));
    }
    const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform(joint_names.at(0));
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);
    Q_EMIT getRobotModelFrame_signal(ROBOT_MODEL_FRAME_,end_effector);
  }
  else
  {

    ROS_INFO("End effector link is not empty");
    const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform(moveit_group_->getEndEffectorLink());
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);

    Q_EMIT getRobotModelFrame_signal(ROBOT_MODEL_FRAME_,end_effector);
  }

}
void GenerateCartesianPath::moveToHome()
{

  geometry_msgs::Pose home_pose;
  tf::poseTFToMsg(end_effector,home_pose);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(home_pose);

  cartesianPathHandler(waypoints, false);

}

void GenerateCartesianPath::getSelectedGroupIndex(int index)
{
  selected_plan_group = index;

  ROS_INFO_STREAM("selected name is:"<<group_names[selected_plan_group]);
  moveit_group_.reset();
  kinematic_state_.reset();
  moveit_group_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(group_names[selected_plan_group]));

  kinematic_state_ = moveit::core::RobotStatePtr(new robot_state::RobotState(kmodel_));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kmodel_->getJointModelGroup(group_names[selected_plan_group]);

  ROS_INFO("End effector link is not empty");
  const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform(moveit_group_->getEndEffectorLink());
  tf::transformEigenToTF(end_effector_state, end_effector);

  Q_EMIT getRobotModelFrame_signal(ROBOT_MODEL_FRAME_, end_effector);

}

void GenerateCartesianPath::ChangeCheckIk(){
  check_ik = !check_ik;
  ROS_INFO_STREAM("Check IK set to: "<< std::to_string(check_ik));
}