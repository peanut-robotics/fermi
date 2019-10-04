#include <moveit_cartesian_plan_plugin/widgets/path_planning_widget.hpp>
#include <moveit_cartesian_plan_plugin/point_tree_model.hpp>
#include <moveit_cartesian_plan_plugin/generate_cartesian_path.hpp>

namespace moveit_cartesian_plan_plugin
{
namespace widgets
{

PathPlanningWidget::PathPlanningWidget(std::string ns) :

                                                         param_ns_(ns)
{
  robot_goal_pub = nh_.advertise<moveit_msgs::DisplayRobotState>("arm_goal_state", 20);
  get_clean_path_proxy_ = nh_.serviceClient<peanut_cotyledon::GetCleanPath>("/oil/cotyledon/get_clean_path", 20);
  set_clean_path_proxy_ = nh_.serviceClient<peanut_cotyledon::SetCleanPath>("/oil/cotyledon/set_clean_path", 20);
  get_objects_proxy_ = nh_.serviceClient<peanut_cotyledon::GetObjects>("/oil/cotyledon/get_objects", 20);
  set_objects_proxy_ = nh_.serviceClient<peanut_cotyledon::SetObjects>("/oil/cotyledon/set_objects", 20);
  get_tasks_proxy_ = nh_.serviceClient<peanut_cotyledon::GetTasks>("/oil/cotyledon/get_tasks", 20);
  set_tasks_proxy_ = nh_.serviceClient<peanut_cotyledon::SetTasks>("/oil/cotyledon/set_tasks", 20);

  move_elevator_ = boost::shared_ptr<actionlib::SimpleActionClient<peanut_elevator_oil::MoveToHeightAction>>(new actionlib::SimpleActionClient<peanut_elevator_oil::MoveToHeightAction>(nh_, "/oil/elevator/move_to_height", true));
  move_base_ = boost::shared_ptr<actionlib::SimpleActionClient<peanut_navplanning_oil::MoveBaseAction>>(new actionlib::SimpleActionClient<peanut_navplanning_oil::MoveBaseAction>(nh_, "/oil/navigation/planning/move_base", true));
  
  // Kortex services
  clear_faults_ = nh_.serviceClient<kortex_driver::ClearFaults>("/resources/manipulation/control/ClearFaults", 20);
  switch_controllers_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("/resources/manipulation/control/controller_manager/switch_controller", 20);
  /*! Constructor which calls the init() function.
      */
  init();
}
PathPlanningWidget::~PathPlanningWidget()
{
}
void PathPlanningWidget::init()
{

  /*! Initializing the RQT UI. Setting up the default values for the UI components:
            - Default Values for the MoveIt and Cartesian Path
            - Validators for the MoveIt and Cartesian Path entries
              - the MoveIt planning time allowed range is set from 1.0 to 1000.0 seconds
              - the MoveIt StepSize allowed range is set from 0.001 to 0.1 meters
              - the Jump Threshold for the IK solutions is set from 0.0 to 10000.0
              .
            .
      */
  ui_.setupUi(this);

  ui_.txtPointName->setText("0");
  //set up the default values for the MoveIt and Cartesian Path
  ui_.lnEdit_PlanTime->setText("5.0");
  ui_.lnEdit_StepSize->setText("0.01");
  ui_.lnEdit_JmpThresh->setText("0.0");

  //set validators for the entries
  ui_.lnEdit_PlanTime->setValidator(new QDoubleValidator(1.0, 100.0, 2, ui_.lnEdit_PlanTime));
  ui_.lnEdit_StepSize->setValidator(new QDoubleValidator(0.001, 0.1, 3, ui_.lnEdit_StepSize));
  ui_.lnEdit_JmpThresh->setValidator(new QDoubleValidator(0.0, 1000.0, 3, ui_.lnEdit_JmpThresh));

  //set progress bar when loading way-points from a yaml file. Could be nice when loading large way-points files
  ui_.progressBar->setRange(0, 100);
  ui_.progressBar->setValue(0);
  ui_.progressBar->hide();

  QStringList headers;
  headers << tr("Point") << tr("Position (m)") << tr("Orientation (deg)");
  ui_.btn_LoadPath->setToolTip(tr("Load Way-Points from a file"));
  ui_.btn_SavePath->setToolTip(tr("Save Way-Points to a file"));
  ui_.btnAddPoint->setToolTip(tr("Add a new Way-Point"));
  ui_.btnRemovePoint->setToolTip(tr("Remove a selected Way-Point"));

  connect(ui_.targetPoint, SIGNAL(clicked()), this, SLOT(sendCartTrajectoryParamsFromUI()));
  connect(ui_.targetPoint, SIGNAL(clicked()), this, SLOT(parseWayPointBtn_slot()));
  connect(ui_.playSubset_btn, SIGNAL(clicked()), this, SLOT(playUntilPointBtn()));
  connect(ui_.playSubset_btn, SIGNAL(clicked()), this, SLOT(sendCartTrajectoryParamsFromUI()));

  connect(ui_.prev_btn, SIGNAL(clicked()), this, SLOT(goToPrev()));
  connect(ui_.next_btn, SIGNAL(clicked()), this, SLOT(goToNext()));

  connect(ui_.btn_plan_config, SIGNAL(clicked()), this, SLOT(parsePlanConfigBtn_slot()));
  connect(ui_.btn_planexecute_config, SIGNAL(clicked()), this, SLOT(parsePlanExecuteConfigBtn_slot()));
  connect(ui_.btn_planexecute_config, SIGNAL(clicked()), this, SLOT(sendCartTrajectoryParamsFromUI()));
  connect(ui_.LineEdit_j1, SIGNAL(editingFinished()), this, SLOT(visualizeGoalConfig()));
  connect(ui_.LineEdit_j2, SIGNAL(editingFinished()), this, SLOT(visualizeGoalConfig()));
  connect(ui_.LineEdit_j3, SIGNAL(editingFinished()), this, SLOT(visualizeGoalConfig()));
  connect(ui_.LineEdit_j4, SIGNAL(editingFinished()), this, SLOT(visualizeGoalConfig()));
  connect(ui_.LineEdit_j5, SIGNAL(editingFinished()), this, SLOT(visualizeGoalConfig()));
  connect(ui_.LineEdit_j6, SIGNAL(editingFinished()), this, SLOT(visualizeGoalConfig()));
  connect(ui_.LineEdit_j7, SIGNAL(editingFinished()), this, SLOT(visualizeGoalConfig()));

  connect(ui_.btn_LoadPath, SIGNAL(clicked()), this, SLOT(loadPoints()));
  connect(ui_.btn_SavePath, SIGNAL(clicked()), this, SLOT(savePoints()));
  connect(ui_.btn_ClearAllPoints, SIGNAL(clicked()), this, SLOT(clearAllPoints_slot()));
  connect(ui_.btn_ClearAllBoxes, SIGNAL(clicked()), this, SLOT(clearAllInteractiveBoxes_slot()));

  //connect(ui_.transform_robot_model_frame, SIGNAL(clicked()), this, SLOT(transformPointsToFrame()));

  connect(ui_.combo_planGroup, SIGNAL(currentIndexChanged(int)), this, SLOT(selectedPlanGroup(int)));

  connect(ui_.mv_el, SIGNAL(clicked()), this, SLOT(moveElevator()));
  connect(ui_.save_pose, SIGNAL(clicked()), this, SLOT(addNavPose()));
  connect(ui_.move_to_pose, SIGNAL(clicked()), this, SLOT(goToNavPose()));

  connect(ui_.clear_faults_btn, SIGNAL(clicked()), this, SLOT(clearFaults()));
  connect(ui_.stop_all_btn, SIGNAL(clicked()), this, SLOT(stopAll()));
  connect(ui_.start_controller_btn, SIGNAL(clicked()), this, SLOT(startController()));

  connect(ui_.btn_checkIK, SIGNAL(clicked()), this, SLOT(ChangeCheckIK()));
  connect(ui_.btn_checkAllIK, SIGNAL(clicked()), this, SLOT(CheckAllPointsIK()));
  connect(ui_.step_size_btn, SIGNAL(clicked()), this, SLOT(ChangeStepSize()));
  connect(ui_.btn_ik_planning, SIGNAL(clicked()), this, SLOT(RobotIKPlanning()));

  connect(ui_.set_tool_btn, SIGNAL(clicked()), this, SLOT(SetTool()));
  connect(ui_.set_mesh_btn, SIGNAL(clicked()), this, SLOT(SetMesh()));
}

void PathPlanningWidget::getCartPlanGroup(std::vector<std::string> group_names)
{
  ROS_INFO("setting the name of the planning group in combo box");
  int lenght_group = group_names.size();

  for (int i = 0; i < lenght_group; i++)
  {
    ui_.combo_planGroup->addItem(QString::fromStdString(group_names[i]));
  }
}

void PathPlanningWidget::selectedPlanGroup(int index)
{
  Q_EMIT sendSendSelectedPlanGroup(index);
}

void PathPlanningWidget::sendCartTrajectoryParamsFromUI()
{
  /*! This function takes care of sending the User Entered parameters from the RQT to the Cartesian Path Planner.
        */
  double plan_time_, cart_step_size_, cart_jump_thresh_;
  bool moveit_replan_, avoid_collisions_, fix_start_state_;

  plan_time_ = ui_.lnEdit_PlanTime->text().toDouble();
  cart_step_size_ = ui_.lnEdit_StepSize->text().toDouble();
  cart_jump_thresh_ = ui_.lnEdit_JmpThresh->text().toDouble();

  moveit_replan_ = ui_.chk_AllowReplanning->isChecked();
  avoid_collisions_ = ui_.chk_AvoidColl->isChecked();
  fix_start_state_ = ui_.chk_FixStartState->isChecked();
  std::string robot_model_frame = ui_.robot_model_frame->text().toStdString();

  Q_EMIT cartesianPathParamsFromUI_signal(plan_time_, cart_step_size_, cart_jump_thresh_, moveit_replan_, avoid_collisions_, robot_model_frame, fix_start_state_);
}

void PathPlanningWidget::parseWayPointBtn_slot()
{
  /*! Letting know the Cartesian Path Planner Class that the user has pressed the Execute Cartesian Path button.
      */
  Q_EMIT parseWayPointBtn_signal();
}
void PathPlanningWidget::goToPrev(){
  int start_idx = ui_.start_idx->text().toInt();
  int stop_idx = ui_.stop_idx->text().toInt();

  start_idx-=1;
  stop_idx-=1;

  ui_.start_idx->setText(QString::number(start_idx));
  ui_.stop_idx->setText(QString::number(stop_idx));
  playUntilPointBtn();
}
void PathPlanningWidget::goToNext(){
  int start_idx = ui_.start_idx->text().toInt();
  int stop_idx = ui_.stop_idx->text().toInt();

  start_idx+=1;
  stop_idx+=1;

  ui_.start_idx->setText(QString::number(start_idx));
  ui_.stop_idx->setText(QString::number(stop_idx));
  playUntilPointBtn();
}
void PathPlanningWidget::playUntilPointBtn()
{
  /*! Let the cartesain planner class plan from start to stop index
      */
  int start_idx = ui_.start_idx->text().toInt();
  int stop_idx = ui_.stop_idx->text().toInt();

  Q_EMIT parseWayPointBtnGoto_signal(start_idx, stop_idx);
}
void PathPlanningWidget::parsePlanConfigBtn_slot()
{
  std::vector<double> config;

  config.push_back(ui_.LineEdit_j1->text().toDouble());
  config.push_back(ui_.LineEdit_j2->text().toDouble());
  config.push_back(ui_.LineEdit_j3->text().toDouble());
  config.push_back(ui_.LineEdit_j4->text().toDouble());
  config.push_back(ui_.LineEdit_j5->text().toDouble());
  config.push_back(ui_.LineEdit_j6->text().toDouble());
  config.push_back(ui_.LineEdit_j7->text().toDouble());

  bool plan_only = true;
  Q_EMIT parseConfigBtn_signal(config, plan_only);
}

void PathPlanningWidget::visualizeGoalConfig()
{
  try
  {
    ROS_DEBUG("Visualizing goal configuration");

    std::vector<double> config;

    moveit_msgs::DisplayRobotState rstate;
    std::vector<std::string> link_names;
    link_names = {"base_link", "shoulder_link", "half_arm_1_link", "half_arm_2_link", "forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_link", "end_effector_link"};

    std_msgs::ColorRGBA highlight_color;
    highlight_color.a = 0.5;
    highlight_color.r = 0;
    highlight_color.g = 1;
    highlight_color.b = 0;

    std::vector<moveit_msgs::ObjectColor> highlight_links;
    for (auto const &link_name : link_names)
    {
      moveit_msgs::ObjectColor o_color;
      o_color.color = highlight_color;
      o_color.id = link_name;
      highlight_links.push_back(o_color);
    }
    ros::Duration timeout_dur(1.0);
    sensor_msgs::JointStateConstPtr joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", timeout_dur);
    std::map<std::string, double> m;
    assert(joint_state->position.size() == joint_state->name.size());
    for (size_t i = 0; i < joint_state->name.size(); ++i)
    {
      m[joint_state->name[i]] = joint_state->position[i];
    }

    m["joint_1"] = ui_.LineEdit_j1->text().toDouble();
    m["joint_2"] = ui_.LineEdit_j2->text().toDouble();
    m["joint_3"] = ui_.LineEdit_j3->text().toDouble();
    m["joint_4"] = ui_.LineEdit_j4->text().toDouble();
    m["joint_5"] = ui_.LineEdit_j5->text().toDouble();
    m["joint_6"] = ui_.LineEdit_j6->text().toDouble();
    m["joint_7"] = ui_.LineEdit_j7->text().toDouble();

    std::vector<double> positions;
    std::vector<std::string> joint_names;
    for (std::map<std::string, double>::iterator it = m.begin(); it != m.end(); ++it)
    {
      joint_names.push_back(it->first);
      positions.push_back(it->second);
    }
    rstate.highlight_links = highlight_links;
    rstate.state.is_diff = false;
    rstate.state.joint_state.position = positions;
    rstate.state.joint_state.name = joint_names;
    rstate.state.joint_state.header.frame_id = "/odom";

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("map", "mobile_base_link", ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Failed to get transform between map and odom for goal state visualization, error: " << ex.what());
      return;
    }

    rstate.state.multi_dof_joint_state.header.frame_id = "/odom";
    rstate.state.multi_dof_joint_state.joint_names = {"odom_virtual_joint"};
    rstate.state.multi_dof_joint_state.transforms = {transformStamped.transform};
    robot_goal_pub.publish(rstate);
    Q_EMIT configEdited_signal(config);
  }
  catch (...)
  {
    ROS_ERROR("Got an error while trying to visualize goal config");
  }
}

void PathPlanningWidget::parsePlanExecuteConfigBtn_slot()
{
  std::vector<double> config;

  config.push_back(ui_.LineEdit_j1->text().toDouble());
  config.push_back(ui_.LineEdit_j2->text().toDouble());
  config.push_back(ui_.LineEdit_j3->text().toDouble());
  config.push_back(ui_.LineEdit_j4->text().toDouble());
  config.push_back(ui_.LineEdit_j5->text().toDouble());
  config.push_back(ui_.LineEdit_j6->text().toDouble());
  config.push_back(ui_.LineEdit_j7->text().toDouble());

  bool plan_only = false;
  Q_EMIT parseConfigBtn_signal(config, plan_only);
}

void PathPlanningWidget::loadPointsTool(){
    /*! Slot that takes care of opening a previously saved Way-Points yaml file.
              Opens Qt Dialog for selecting the file, opens the file and parses the data.
              After reading and parsing the data from the file, the information regarding the pose of the Way-Points is send to the RQT and the RViz so they can update their enviroments.
          */

  QString fileName = QFileDialog::getOpenFileName(this,
                                                  tr("Open Way Points File"), "",
                                                  tr("Way Points (*.yaml);;All Files (*)"));

  if (fileName.isEmpty())
  {
    ui_.tabWidget->setEnabled(true);
    ui_.progressBar->hide();
    return;
  }
  else
  {
    ui_.tabWidget->setEnabled(false);
    ui_.progressBar->show();
    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly))
    {
      QMessageBox::information(this, tr("Unable to open file"),
                              file.errorString());
      file.close();
      ui_.tabWidget->setEnabled(true);
      ui_.progressBar->hide();
      return;
    }
    //clear all the scene before loading all the new points from the file!!
    clearAllPoints_slot();

    ROS_INFO_STREAM("Opening the file: " << fileName.toStdString());
    std::string fin(fileName.toStdString());
    std::string frame_id;
    try
    {
      YAML::Node doc;
      doc = YAML::LoadFile(fin);
      //define double for percent of completion
      double percent_complete;
      int end_of_points = doc["points"].size();

      std::vector<double> startConfig = doc["start_config"].as<std::vector<double>>();

      ui_.LineEdit_j1->setText(QString::number(startConfig.at(0)));
      ui_.LineEdit_j2->setText(QString::number(startConfig.at(1)));
      ui_.LineEdit_j3->setText(QString::number(startConfig.at(2)));
      ui_.LineEdit_j4->setText(QString::number(startConfig.at(3)));
      ui_.LineEdit_j5->setText(QString::number(startConfig.at(4)));
      ui_.LineEdit_j6->setText(QString::number(startConfig.at(5)));
      ui_.LineEdit_j7->setText(QString::number(startConfig.at(6)));
      std::cout << end_of_points << "end of doc" << std::endl;
      frame_id = doc["frame_id"].as<std::string>();
      for (size_t i = 0; i < end_of_points; i++)
      {
        std::string name;
        geometry_msgs::Pose pose;
        tf::Transform pose_tf;

        double x, y, z;
        double qx, qy, qz, qw;

        name = std::to_string(i);
        x = doc["points"][i]["position"]["x"].as<double>();
        y = doc["points"][i]["position"]["y"].as<double>();
        z = doc["points"][i]["position"]["z"].as<double>();
        qx = doc["points"][i]["orientation"]["x"].as<double>();
        qy = doc["points"][i]["orientation"]["y"].as<double>();
        qz = doc["points"][i]["orientation"]["z"].as<double>();
        qw = doc["points"][i]["orientation"]["w"].as<double>();

        pose_tf = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z));

        percent_complete = (i + 1) * 100 / end_of_points;
        ui_.progressBar->setValue(percent_complete);
        Q_EMIT addPoint(pose_tf);
        Q_EMIT configEdited_signal(startConfig);
      }
    }
    catch (char *excp)
    {
      ROS_INFO("bla de bla, first error");
      ROS_INFO_STREAM("Caught " << excp);
    }
    catch (...)
    {
      ROS_ERROR("Not able to load file yaml, might be incorrectly formatted");
    }
    // TODO call same pathway as button
    ui_.tabWidget->setEnabled(true);
    ui_.progressBar->hide();
    ui_.robot_model_frame->setText(QString::fromStdString(frame_id));
    PathPlanningWidget::transformPointsToFrame();
  }
}

void PathPlanningWidget::loadPointsObject()
{
  /* Slot that takes care of opening a previously saved Way-Points yaml file.
    Opens Qt Dialog for selecting the file, opens the file and parses the data.
    After reading and parsing the data from the file, the information regarding 
    the pose of the Way-Points is send to the RQT and the RViz so they can update 
    their enviroments.*/

  const std::string frame_id = "map";
  double elevator_height, percent_complete ;
  int num_robot_poses; 

  // Clean path data
  std::string floor_name = ui_.floor_name_line_edit->text().toStdString();
  std::string area_name = ui_.area_name_line_edit->text().toStdString();
  int object_id = ui_.object_id_line_edit->text().toInt();
  std::string task_name = ui_.task_name_line_edit->text().toStdString();
  peanut_cotyledon::CleanPath clean_path;
  peanut_cotyledon::Object desired_object;

  // Transforms
  geometry_msgs::Transform object_world_tfmsg;
  tf::Transform object_world_tf;

  // Clear all the scene before loading all the new points from the file!!
  clearAllPoints_slot();

  // Enforce target frame "map"
  ui_.robot_model_frame->setText(QString::fromStdString(frame_id));
  PathPlanningWidget::transformPointsToFrame();

  // Get clean path
  peanut_cotyledon::GetCleanPath srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  srv.request.object_id = object_id;
  srv.request.task_name = task_name;

  if(get_clean_path_proxy_.call(srv))
  {
    clean_path = srv.response.clean_path;
  }
  else
  {
    ROS_ERROR_STREAM("clean path floor" << floor_name << "area" << area_name << "object_id" << std::to_string(object_id) << "task_name" << task_name << "not able to load");
    ui_.tabWidget->setEnabled(true);
    ui_.progressBar->hide();
    return;
  }

  if (clean_path.cached_paths.empty() || clean_path.object_poses.empty())
  {
    ui_.tabWidget->setEnabled(true);
    ui_.progressBar->hide();
    ROS_ERROR("No object poses or cache paths in clean_path");
    return;
  }
  
  ui_.tabWidget->setEnabled(false);
  ui_.progressBar->show();
  num_robot_poses = clean_path.object_poses.size();
  
  // Get object transform
  if (!getObjectWithID(floor_name, area_name, object_id, desired_object)){
    ROS_ERROR_STREAM("Could not find object with ID"<<object_id);
    return;
  }
  object_world_tfmsg = desired_object.origin;
  tf::transformMsgToTF(object_world_tfmsg, object_world_tf);

  // Load object poses and update ui/server points
  int i = 0;
  tf::Transform pose_tf;
  tf::Quaternion q;

  ROS_INFO_STREAM("Loading clean path with "<<clean_path.object_poses.size()<< " points");
  for (auto pose : clean_path.object_poses) // object_poses are in object frame
  {
    i++;
    tf::poseMsgToTF(pose, pose_tf);
    pose_tf = object_world_tf * pose_tf;
    
    // Normalize 
    q = pose_tf.getRotation();
    q = q.normalize();
    pose_tf.setRotation(q);
    
    // if( std::isnan(pose_tf.getOrigin()[0]) || std::isnan(pose_tf.getOrigin()[1]) || std::isnan(pose_tf.getOrigin()[2]) ||
    //     std::isnan(pose_tf.getRotation().getW()) || std::isnan(pose_tf.getRotation().getAxis()[0]) || 
    //     std::isnan(pose_tf.getRotation().getAxis()[1]) || std::isnan(pose_tf.getRotation().getAxis()[2])){
    //       ROS_ERROR("Points contains NaNs. Cannot load path");
    //       return;
    // }

    // if( std::isinf(pose_tf.getOrigin()[0]) || std::isinf(pose_tf.getOrigin()[1]) || std::isinf(pose_tf.getOrigin()[2]) ||
    //     std::isinf(pose_tf.getRotation().getW()) || std::isinf(pose_tf.getRotation().getAxis()[0]) || 
    //     std::isinf(pose_tf.getRotation().getAxis()[1]) || std::isinf(pose_tf.getRotation().getAxis()[2])){
    //       ROS_ERROR("Points contains Inf. Cannot load path");
    //       return;
    // }

    percent_complete = (i + 1) * 100 / num_robot_poses;
    ui_.progressBar->setValue(percent_complete);
    Q_EMIT addPoint(pose_tf);
  }

  // Set starting config 
  if (!clean_path.cached_paths.at(0).cached_path.points.empty()){
      ROS_INFO("Cache path is present, loading starting config");
      std::vector<double> startConfig = clean_path.cached_paths[0].cached_path.points[0].positions;
      ui_.LineEdit_j1->setText(QString::number(startConfig.at(0)));
      ui_.LineEdit_j2->setText(QString::number(startConfig.at(1)));
      ui_.LineEdit_j3->setText(QString::number(startConfig.at(2)));
      ui_.LineEdit_j4->setText(QString::number(startConfig.at(3)));
      ui_.LineEdit_j5->setText(QString::number(startConfig.at(4)));
      ui_.LineEdit_j6->setText(QString::number(startConfig.at(5)));
      ui_.LineEdit_j7->setText(QString::number(startConfig.at(6)));
      Q_EMIT configEdited_signal(startConfig);
  }

  // Get elevator height
  elevator_height = clean_path.cached_paths.at(0).elevator_height;

  try
  {
    ROS_INFO("Setting step size and frame id");
    ui_.el_lbl->setText(QString::number(elevator_height));
    ui_.robot_model_frame->setText(QString::fromStdString(frame_id));
    ui_.lnEdit_StepSize->setText(QString::fromStdString(std::to_string(clean_path.max_step)));
    ui_.chk_AvoidColl->setChecked(clean_path.avoid_collisions);
    ui_.lnEdit_JmpThresh->setText(QString::fromStdString(std::to_string(clean_path.jump_threshold)));
    ui_.tool_name_lbl->setText(QString::fromStdString(clean_path.tool_name));
  }
  catch (...)
  {
    ROS_ERROR("Not able to load file yaml, might be incorrectly formatted");
  }

  // TODO call same pathway as button
  ui_.tabWidget->setEnabled(true);
  ui_.progressBar->hide();

  // Get obj info and update control
  std::string mesh_name = desired_object.geometry_path[0];
  geometry_msgs::Pose object_pose;
  object_pose.position.x = desired_object.origin.translation.x;
  object_pose.position.y = desired_object.origin.translation.y;
  object_pose.position.z = desired_object.origin.translation.z;
  object_pose.orientation = desired_object.origin.rotation;
  // #TODO this call updates parent_home as well. Add a function to update parent_home only.
  Q_EMIT modifyMarkerControl_signal(mesh_name, object_pose);
  
  // Update mesh text field 
  ui_.mesh_name_lbl->setText(QString::fromStdString(mesh_name));
  ROS_INFO("Completed load process for clean path");
}

void PathPlanningWidget::savePoints(){
  if (ui_.chk_istoolpath->isChecked()){
    PathPlanningWidget::savePointsTool();
  }
  else if(ui_.chk_isRefPose->isChecked()){
    PathPlanningWidget::saveRefNavPose();
  }
  else{
    PathPlanningWidget::savePointsObject();
  }

}
void PathPlanningWidget::loadPoints(){
  if (ui_.chk_istoolpath->isChecked()){
    PathPlanningWidget::loadPointsTool();
  }else{
    PathPlanningWidget::loadPointsObject();
  }
}
void PathPlanningWidget::savePointsTool(){
  ROS_INFO("Begin saving tool path to file");
  Q_EMIT saveToolBtn_press();
}

void PathPlanningWidget::saveRefNavPose(){
  std::string floor_name = ui_.floor_name_line_edit->text().toStdString();
  std::string area_name = ui_.area_name_line_edit->text().toStdString();
  int object_id = ui_.object_id_line_edit->text().toInt();
  std::string task_name = ui_.task_name_line_edit->text().toStdString();
  std::string mesh_name = ui_.mesh_name_lbl->text().toStdString();
  peanut_cotyledon::CleanPath clean_path;

  // Check task data
  if (object_id != 0){
    ROS_ERROR_STREAM("Cannot save reference nav pose to object with ID: << "<<object_id<<". ID can only be 0");
    return;
  }

  // Check if task exists
  peanut_cotyledon::GetTasks tasks_srv;
  peanut_cotyledon::Task desired_task;
  tasks_srv.request.floor_name = floor_name;
  tasks_srv.request.area_name = area_name;
  tasks_srv.request.object_id = object_id;
  if (!get_tasks_proxy_.call(tasks_srv)){
    ROS_ERROR("Could not call get_tasks service");
    return;
  }

  // Find task
  bool found_task = false;
  for(auto& task: tasks_srv.response.tasks){
    if (task.name == task_name){
      desired_task = task;
      found_task = true;
      break;
    }
  }

  if(found_task){
    // Check task type
    if(desired_task.task_type != peanut_cotyledon::Task::NAVIGATE){
      ROS_ERROR_STREAM("Existing task: "<<desired_task.name<<" is not of type NAVIGATE");
      return;
    }
  }
  else{
    // Create and add new task
    ROS_INFO_STREAM("Creating and adding new task "<<task_name);
    peanut_cotyledon::Task new_task;
    new_task.name = task_name;
    new_task.task_type = peanut_cotyledon::Task::NAVIGATE;

    peanut_cotyledon::SetTasks set_tasks_srv;
    set_tasks_srv.request.floor_name = floor_name;
    set_tasks_srv.request.area_name = area_name;
    set_tasks_srv.request.object_id = object_id;
    set_tasks_srv.request.tasks = tasks_srv.response.tasks;
    set_tasks_srv.request.tasks.push_back(new_task);

    if (set_tasks_proxy_.call(set_tasks_srv)){
      if (!set_tasks_srv.response.success){
        ROS_ERROR_STREAM("set_tasks service failed: "<<set_tasks_srv.response.message);
        return;
      }
    }
    else{
      ROS_ERROR("Could not call set_tasks service");
      return;
    }
  }
  
  // Get clean path for this path
  peanut_cotyledon::GetCleanPath clean_path_srv;
  clean_path_srv.request.floor_name = floor_name;
  clean_path_srv.request.area_name = area_name;
  clean_path_srv.request.object_id = object_id;
  clean_path_srv.request.task_name = task_name;
  if(get_clean_path_proxy_.call(clean_path_srv))
  {
    clean_path = clean_path_srv.response.clean_path;
  }
  else
  {
    ROS_ERROR_STREAM("Could not call get_clean_path service");
    return;
  }

  // Add a cached path if it does not exist
  if(clean_path.cached_paths.size() == 0){
    peanut_cotyledon::CachedPath cach_path;
    clean_path.cached_paths.push_back(cach_path);
  }

  // Save Nav pose
  addNavPoseHelper();
}

void PathPlanningWidget::savePointsObject()
{
  /*! Just inform the RViz enviroment that Save Way-Points button has been pressed.
       */
  std::string floor_name = ui_.floor_name_line_edit->text().toStdString();
  std::string area_name = ui_.area_name_line_edit->text().toStdString();
  int object_id = ui_.object_id_line_edit->text().toInt();
  std::string task_name = ui_.task_name_line_edit->text().toStdString();
  std::string mesh_name = ui_.mesh_name_lbl->text().toStdString();
  peanut_cotyledon::CleanPath clean_path;

  // Get clean path
  peanut_cotyledon::GetCleanPath srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  srv.request.object_id = object_id;
  srv.request.task_name = task_name;
  if(get_clean_path_proxy_.call(srv))
  {
    clean_path = srv.response.clean_path;
  }
  else
  {
    ROS_INFO_STREAM("clean path floor" << floor_name << "area" << area_name << "object_id" << std::to_string(object_id) << "task_name" << task_name << "not able to load");
    return;
  }

  if (clean_path.cached_paths.empty()){
    peanut_cotyledon::CachedPath empty_cached_path;
    std::vector<peanut_cotyledon::CachedPath> empty_cached_path_list;
    empty_cached_path_list.push_back(empty_cached_path);
    clean_path.cached_paths = empty_cached_path_list;
  }

  /*
  The following data is saved/overwritten here
  - Elevator height
  - Max step size
  - Jump threshold
  - Tool name 
  */
  clean_path.cached_paths.at(0).elevator_height = ui_.el_lbl->text().toDouble();
  clean_path.max_step = ui_.lnEdit_StepSize->text().toDouble();
  clean_path.avoid_collisions = ui_.chk_AvoidColl->isChecked();
  clean_path.jump_threshold = ui_.lnEdit_JmpThresh->text().toDouble();
  clean_path.tool_name = ui_.tool_name_lbl->text().toStdString();

  Q_EMIT saveObjectBtn_press(floor_name, area_name, object_id, task_name, clean_path, mesh_name);
}
void PathPlanningWidget::transformPointsToFrame()
{
  PathPlanningWidget::sendCartTrajectoryParamsFromUI();
  //TODO get frame name
  std::string frame = ui_.robot_model_frame->text().toStdString();

  Q_EMIT transformPointsViz(frame);
}
void PathPlanningWidget::clearAllPoints_slot()
{
  /*! Clear all the Way-Points from the RViz enviroment
      */
  tf::Transform t;
  t.setIdentity();

  Q_EMIT clearAllPoints_signal();
}

void PathPlanningWidget::clearAllInteractiveBoxes_slot()
{
  Q_EMIT clearAllInteractiveBoxes_signal();
}

void PathPlanningWidget::cartesianPathStartedHandler()
{
  /*! Disable the RQT Widget when the Cartesian Path is executing.
      */
  ui_.tabWidget->setEnabled(false);
  ui_.targetPoint->setEnabled(false);
}
void PathPlanningWidget::cartesianPathFinishedHandler()
{
  /*! Enable the RQT Widget when the Cartesian Path execution is completed.
      */
  ui_.tabWidget->setEnabled(true);
  ui_.targetPoint->setEnabled(true);
}
void PathPlanningWidget::cartPathCompleted_slot(double fraction)
{
  /*! Get the information of what is the percentage of completion of the Planned Cartesian path from the Cartesian Path Planner class and display it in Qt label.
      */
  fraction = fraction * 100.0;
  fraction = std::floor(fraction * 100 + 0.5) / 100;

  ui_.lbl_cartPathCompleted->setText("Cartesian path " + QString::number(fraction) + "% completed.");
}

void PathPlanningWidget::moveToHomeFromUI()
{
  Q_EMIT moveToHomeFromUI_signal();
}

void PathPlanningWidget::moveElevator()
{
  QFuture<void> future = QtConcurrent::run(this, &PathPlanningWidget::moveElevatorHelper);
}

void PathPlanningWidget::moveElevatorHelper()
{ 
  double height = ui_.el_lbl->text().toDouble();
  peanut_elevator_oil::MoveToHeightGoal goal;
  goal.height = height;

  move_elevator_->sendGoal(goal);
  bool success = move_elevator_->waitForResult(ros::Duration(60.0));

  if (success){
    ROS_INFO_STREAM("Elevator moved to height "<<std::to_string(height));
  
  }
  else{
    ROS_ERROR("Failed to move elevator");
  }
}

void PathPlanningWidget::addNavPose()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(this, "Save", "Save Nav Task Pose?", QMessageBox::Yes|QMessageBox::No);
  if (reply == QMessageBox::Yes){
    ROS_INFO("Saving Nav Pose");
  QFuture<void> future = QtConcurrent::run(this, &PathPlanningWidget::addNavPoseHelper);
}
  else{
    ROS_INFO("Did not save Nav Pose");
    return;
  }
}

void PathPlanningWidget::addNavPoseHelper()
{ 
  // Get data
  std::string floor_name = ui_.floor_name_line_edit->text().toStdString();
  std::string area_name = ui_.area_name_line_edit->text().toStdString();
  int object_id = ui_.object_id_line_edit->text().toInt();
  std::string task_name = ui_.task_name_line_edit->text().toStdString();

  // Transforms 
  tf2_ros::TransformListener tfListener(tfBuffer_);
  geometry_msgs::Transform robot_world_tf;
  geometry_msgs::Transform object_world_tf;

  Eigen::Affine3d object_world_eigen;
  Eigen::Affine3d robot_object_eigen;
  Eigen::Affine3d robot_world_eigen;

  geometry_msgs::Quaternion quat_msg;

  // Get objects
  bool found_tf = false;
  std::string obj_name;
  peanut_cotyledon::GetObjects srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  if (get_objects_proxy_.call(srv)){
    for(auto& obj : srv.response.objects){
      if(obj.id == object_id){
        object_world_tf = obj.origin;
        tf::transformMsgToEigen (object_world_tf, object_world_eigen);
        found_tf = true;
        break;
      }
    }
  }
  else{
    ROS_ERROR("Could not call get objects service");
    return;
  }

  if(!found_tf){
    ROS_ERROR_STREAM("Could not find object with ID"<<object_id);
    return;
  }

  // Get current robot location
  int count = 0;
  while(true){
    try{
      robot_world_tf = tfBuffer_.lookupTransform("map", "mobile_base_link", ros::Time(0)).transform;
      tf::transformMsgToEigen (robot_world_tf, robot_world_eigen);
      break;
    }
    catch (tf2::TransformException &ex/*tf::TransformException ex*/) {
      ROS_WARN("%s",ex.what());
      count += 1;
      if(count > 5){
        return;
      }
      ros::Duration(1.0).sleep();
    }
  }

  // Robot wrt object 
  robot_object_eigen = object_world_eigen.inverse() * robot_world_eigen;

  // Get clean path
  peanut_cotyledon::CleanPath clean_path;
  peanut_cotyledon::GetCleanPath path_srv;
  path_srv.request.floor_name = floor_name;
  path_srv.request.area_name = area_name;
  path_srv.request.object_id = object_id;
  path_srv.request.task_name = task_name;

  if(get_clean_path_proxy_.call(path_srv))
  {
    clean_path = path_srv.response.clean_path;
  }
  else
  {
    ROS_ERROR_STREAM("clean path floor" << floor_name << "area" << area_name << "object_id" << std::to_string(object_id) << "task_name" << task_name << "not able to load");
    return;
  }

  // Convert Transform to pose and update cached path
  Eigen::Matrix3d rot = robot_object_eigen.linear();
  Eigen::Quaterniond quat(rot);
  tf::quaternionEigenToMsg(quat, quat_msg);

  geometry_msgs::Pose robot_object_pose;
  robot_object_pose.position.x = robot_object_eigen.translation()[0];
  robot_object_pose.position.y = robot_object_eigen.translation()[1];
  robot_object_pose.position.z = robot_object_eigen.translation()[2];
  robot_object_pose.orientation = quat_msg;
  clean_path.cached_paths.at(0).nav_pose = robot_object_pose;

  // Set clean path
  peanut_cotyledon::SetCleanPath set_path_srv;
  set_path_srv.request.floor_name = floor_name;
  set_path_srv.request.area_name = area_name;
  set_path_srv.request.object_id = object_id;
  set_path_srv.request.task_name = task_name;
  set_path_srv.request.clean_path = clean_path;
  
  if(set_clean_path_proxy_.call(set_path_srv)){
    if(set_path_srv.response.success){
      ROS_INFO("Updated nav_pose for path");
    }
    else{
      ROS_ERROR_STREAM("Could not update nav pose. Error: "<<set_path_srv.response.message);
      return;
    }
  }  
  
}

void PathPlanningWidget::goToNavPose(){
  QFuture<void> future = QtConcurrent::run(this, &PathPlanningWidget::goToNavPoseHelper);
}

void PathPlanningWidget::goToNavPoseHelper(){
  
  // Get data
  std::string floor_name = ui_.floor_name_line_edit->text().toStdString();
  std::string area_name = ui_.area_name_line_edit->text().toStdString();
  int object_id = ui_.object_id_line_edit->text().toInt();
  std::string task_name = ui_.task_name_line_edit->text().toStdString();

  // Transforms 
  tf2_ros::TransformListener tfListener(tfBuffer_);
  geometry_msgs::Transform object_world_tf;

  Eigen::Affine3d object_world_eigen;
  Eigen::Affine3d robot_object_eigen;
  Eigen::Affine3d robot_world_eigen;

  geometry_msgs::Quaternion quat_msg;
  
  // Get clean path
  peanut_cotyledon::CleanPath clean_path;
  peanut_cotyledon::GetCleanPath path_srv;
  path_srv.request.floor_name = floor_name;
  path_srv.request.area_name = area_name;
  path_srv.request.object_id = object_id;
  path_srv.request.task_name = task_name;

  if(get_clean_path_proxy_.call(path_srv))
  {
    clean_path = path_srv.response.clean_path;
  }
  else
  {
    ROS_ERROR_STREAM("clean path floor" << floor_name << "area" << area_name << "object_id" << std::to_string(object_id) << "task_name" << task_name << "not able to load");
    return;
  }
  
  // Get object pose 
  std::string obj_name;
  bool found_tf = false;
  peanut_cotyledon::GetObjects srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  if (get_objects_proxy_.call(srv)){
    for(auto& obj : srv.response.objects){
      if(obj.id == object_id){
        object_world_tf = obj.origin;
        tf::transformMsgToEigen (object_world_tf, object_world_eigen);
        found_tf = true;
        break;
      }
    }
  }
  else{
    ROS_ERROR("Could not call get objects service");
    return;
  }

  if(!found_tf){
    ROS_ERROR_STREAM("Could not find object with ID"<<object_id);
    return;
  }

  // Get object pose and convert to stampedTf
  geometry_msgs::Pose robot_object_pose;
  robot_object_pose = clean_path.cached_paths.at(0).nav_pose;
  tf::poseMsgToEigen (robot_object_pose, robot_object_eigen);

  // Get desired robot wrt world
  robot_world_eigen = object_world_eigen * robot_object_eigen;

  // Send goal
  Eigen::Matrix3d rot = robot_world_eigen.linear();
  Eigen::Quaterniond quat(rot);
  tf::quaternionEigenToMsg(quat, quat_msg); 

  peanut_navplanning_oil::MoveBaseGoal goal;
  goal.goal_pose.header.stamp = ros::Time::now();
  goal.goal_pose.header.frame_id = "map";
  goal.goal_pose.pose.position.x = robot_world_eigen.translation()[0];
  goal.goal_pose.pose.position.y = robot_world_eigen.translation()[1];
  goal.goal_pose.pose.position.z = robot_world_eigen.translation()[2];
  goal.goal_pose.pose.orientation = quat_msg;

  ROS_INFO_STREAM("Sending goal to move base");
  move_base_->sendGoal(goal);
  bool success = move_base_->waitForResult(ros::Duration(60.0));

  if (success){
    ROS_INFO_STREAM("Navigation successfull");
  }
  else{
    ROS_ERROR_STREAM("Navigation failed ");
  }
}

void PathPlanningWidget::clearFaults(){
  kortex_driver::ClearFaults srv;
  
  if (clear_faults_.call(srv)){
    ROS_INFO_STREAM("Clearing faults");
  }
  else{
    ROS_ERROR("Could not call clear faults service");
  }
}

void PathPlanningWidget::startController(){
  // Start Controller
  controller_manager_msgs::SwitchController srv;
  srv.request.stop_controllers = {""};
  srv.request.start_controllers = {"velocity_trajectory_controller"};
  srv.request.strictness = 0;

  if (switch_controllers_.call(srv)){
    if (srv.response.ok){
      ROS_INFO_STREAM("Controller started");
    }
    else{
      ROS_ERROR("Could not start controller");
    }
  }
  else{
    ROS_ERROR("Could not call switch controller service");
  }
}

void PathPlanningWidget::stopController(){
  // Stop Controller
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = {""};
  srv.request.stop_controllers = {"velocity_trajectory_controller"};
  srv.request.strictness = 0;

  if (switch_controllers_.call(srv)){
    if (srv.response.ok){
      ROS_INFO_STREAM("Controller stopped");
    }
    else{
      ROS_ERROR("Could not stop controller");
    }
  }
  else{
    ROS_ERROR("Could not call switch controller service");
  }
}

void PathPlanningWidget::stopAll(){
  // Stop Controller
  stopController();

  // Stop Navigation
  ROS_INFO("Cancelling navigation goal");
  move_base_->cancelAllGoals();
}

void PathPlanningWidget::ChangeCheckIK(){
  Q_EMIT ChangeCheckIK_signal();
}

void PathPlanningWidget::CheckAllPointsIK(){
  Q_EMIT CheckAllPointsIK_signal();
}

bool PathPlanningWidget::getObjectWithID(std::string floor_name, std::string area_name, int object_id, peanut_cotyledon::Object& desired_obj){
  // Get objects
  peanut_cotyledon::GetObjects srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  if (get_objects_proxy_.call(srv)){
    for(auto& obj : srv.response.objects){
      if(obj.id == object_id){
        desired_obj = obj;
        return true;
      }
    }
  }
  else{
    ROS_ERROR("Could not call get objects service");
    return false;
  }
  return false;  
}

bool PathPlanningWidget::setObjectHelper(std::string floor_name, std::string area_name, int object_id, peanut_cotyledon::Object obj){
  // Set objects
  peanut_cotyledon::SetObjects srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  srv.request.objects.push_back(obj);

  if (set_objects_proxy_.call(srv)){
    if(srv.response.success){
      ROS_INFO("Updated mesh name for object");
      return true;
    }
    else{
      ROS_INFO_STREAM("Unable to set mesh name for object.Error: "<<srv.response.message);
    }
  }
  else{
    ROS_ERROR("Could not call set objects service");
    return false;
  } 
}

void PathPlanningWidget::ChangeStepSize(){
  PathPlanningWidget::sendCartTrajectoryParamsFromUI();
}

void PathPlanningWidget::RobotIKPlanning(){
  // Get data
  double upper_limit = ui_.h_upper_limit->text().toDouble();
  double lower_limit = ui_.h_lower_limit->text().toDouble();
  double step_size = ui_.h_step_size->text().toDouble();
  double h = ui_.el_lbl->text().toDouble();
  double radius = ui_.robot_ik_radius->text().toDouble();
  double radius_step = ui_.robot_ik_radius_step->text().toDouble();
  double max_angle = ui_.robot_ik_max_ang->text().toDouble();
  double min_angle = ui_.robot_ik_min_ang->text().toDouble();
  double angle_step = ui_.robot_ik_ang_step->text().toDouble();

  Q_EMIT RobotIKPlanning_signal(upper_limit, lower_limit, step_size, h, radius, radius_step, max_angle, min_angle, angle_step);
}

void PathPlanningWidget::SetTool(){
  std::string tool = ui_.tool_name_lbl->text().toStdString();
  std::string floor_name = ui_.floor_name_line_edit->text().toStdString();
  std::string area_name = ui_.area_name_line_edit->text().toStdString();
  int object_id = ui_.object_id_line_edit->text().toInt();
  std::string task_name = ui_.task_name_line_edit->text().toStdString();
  std::string mesh_name = ui_.mesh_name_lbl->text().toStdString();
  peanut_cotyledon::CleanPath clean_path;

  // Get clean path
  peanut_cotyledon::GetCleanPath srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  srv.request.object_id = object_id;
  srv.request.task_name = task_name;
  if(get_clean_path_proxy_.call(srv))
  {
    clean_path = srv.response.clean_path;
  }
  else
  {
    ROS_INFO_STREAM("clean path floor" << floor_name << "area" << area_name << "object_id" << std::to_string(object_id) << "task_name" << task_name << "not able to load");
    return;
  }

  // Set tool
  clean_path.tool_name = tool;

  // Set clean path
  peanut_cotyledon::SetCleanPath set_path_srv;
  set_path_srv.request.floor_name = floor_name;
  set_path_srv.request.area_name = area_name;
  set_path_srv.request.object_id = object_id;
  set_path_srv.request.task_name = task_name;
  set_path_srv.request.clean_path = clean_path;
  
  if(set_clean_path_proxy_.call(set_path_srv)){
    if(set_path_srv.response.success){
      ROS_INFO_STREAM("Updated tool name to "<<tool);
    }
    else{
      ROS_ERROR_STREAM("Could not update tool. Error: "<<set_path_srv.response.message);
      return;
    }
  }  

}

void PathPlanningWidget::SetMesh(){
  std::string floor_name = ui_.floor_name_line_edit->text().toStdString();
  std::string area_name = ui_.area_name_line_edit->text().toStdString();
  int object_id = ui_.object_id_line_edit->text().toInt();
  std::string task_name = ui_.task_name_line_edit->text().toStdString();
  std::string mesh_name = ui_.mesh_name_lbl->text().toStdString();
  peanut_cotyledon::Object desired_object;

  // Get object transform
  if (!getObjectWithID(floor_name, area_name, object_id, desired_object)){
    ROS_ERROR_STREAM("Could not find object with ID"<<object_id);
    return;
  }

  // Set geometry name
  desired_object.geometry_path.clear();
  desired_object.geometry_path.push_back(mesh_name);

  // Set object
  setObjectHelper(floor_name, area_name, object_id, desired_object);
}


} // namespace widgets
} // namespace moveit_cartesian_plan_plugin
