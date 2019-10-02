
#include <moveit_cartesian_plan_plugin/add_way_point.hpp>

namespace moveit_cartesian_plan_plugin
{

AddWayPoint::AddWayPoint(QWidget *parent) : rviz::Panel(parent) //, tf_()
{
  /*!  The constructor sets the Object name, resets the Interactive Marker server.
         It initialize the subscriber to the mouse click topic and registers the call back to a mouse click event.
         It initializes the constants for the Marker color and scales, all the Interactive Markers are defined as arrows.
         The User Interaction arrow is set to have red color, while the way-points when in an IK Solution are set as blue.
         The Way-Points which are outside an IK solution are set to have yellow color.
    */
  setObjectName("CartesianPathPlannerPlugin");
  server.reset(new interactive_markers::InteractiveMarkerServer("moveit_cartesian_plan_plugin", "", false));

  WAY_POINT_COLOR.r = 0.10;
  WAY_POINT_COLOR.g = 0.20;
  WAY_POINT_COLOR.b = 0.4;
  WAY_POINT_COLOR.a = 1.0;

  WAY_POINT_COLOR_OUTSIDE_IK.r = 1.0;
  WAY_POINT_COLOR_OUTSIDE_IK.g = 1.0;
  WAY_POINT_COLOR_OUTSIDE_IK.b = 0.0;
  WAY_POINT_COLOR_OUTSIDE_IK.a = 1.0;

  WAY_POINT_SCALE_CONTROL.x = 0.08;
  WAY_POINT_SCALE_CONTROL.y = 0.01;
  WAY_POINT_SCALE_CONTROL.z = 0.01;

  INTERACTIVE_MARKER_SCALE = 0.2;

  ARROW_INTER_COLOR.r = 0.8;
  ARROW_INTER_COLOR.g = 0.2;
  ARROW_INTER_COLOR.b = 0.1;
  ARROW_INTER_COLOR.a = 1.0;

  ARROW_INTER_SCALE_CONTROL.x = 0.1;
  ARROW_INTER_SCALE_CONTROL.y = 0.1;
  ARROW_INTER_SCALE_CONTROL.z = 0.1;

  MESH_SCALE_CONTROL.x = 1;
  MESH_SCALE_CONTROL.y = 1;
  MESH_SCALE_CONTROL.z = 1;
  
  CONTROL_MARKER_POSE.position.x = 0;
  CONTROL_MARKER_POSE.position.y = 0;
  CONTROL_MARKER_POSE.position.z = 0;
  CONTROL_MARKER_POSE.orientation.w = 1;
  CONTROL_MARKER_POSE.orientation.x = 0;
  CONTROL_MARKER_POSE.orientation.y = 0;
  CONTROL_MARKER_POSE.orientation.z = 0;

  ARROW_INTERACTIVE_SCALE = 0.2;
  set_clean_path_proxy_ = nh_.serviceClient<peanut_cotyledon::SetCleanPath>("/oil/cotyledon/set_clean_path", 20);
  get_objects_proxy_ = nh_.serviceClient<peanut_cotyledon::GetObjects>("/oil/cotyledon/get_objects", 20);
  set_objects_proxy_ = nh_.serviceClient<peanut_cotyledon::SetObjects>("/oil/cotyledon/set_objects", 20);

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ROS_INFO("Constructor created");
}

AddWayPoint::~AddWayPoint()
{
  /*! The object destructor resets the Interactive Marker server on the Object Destruction. */
  server.reset();
  delete tfListener;
}

void AddWayPoint::onInitialize()
{

  /*!  Creating main layout object, object for the Cartesian Path Planning Class and the RQT Widget.
         Here we also create the Interactive Marker Menu handler for the Way-Points.
         Make all the necessary connections for the QObject communications.
         Inter Object connections for communications between the classes.
     */

  path_generate = new GenerateCartesianPath();
  widget_ = new widgets::PathPlanningWidget("~");
  this->parentWidget()->resize(widget_->width(), widget_->height());
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->addWidget(widget_);
  config = {0, 0, 0, 0, 0, 0, 0};

  //! Inform the user that the RViz is initializing
  ROS_INFO("initializing..");

  menu_handler.insert("delete", boost::bind(&AddWayPoint::processFeedback, this, _1));
  menu_handler.insert("adjust_frame", boost::bind(&AddWayPoint::processFeedback, this, _1));
  menu_handler.insert("adjust_eef", boost::bind(&AddWayPoint::processFeedback, this, _1));
  menu_handler.insert("adjust_hide", boost::bind(&AddWayPoint::processFeedback, this, _1));
  menu_handler.insert("duplicate_in_place", boost::bind(&AddWayPoint::processFeedback, this, _1));

  // menu_handler.insert( "duplicate_at_end", boost::bind( &AddWayPoint::processFeedback, this, _1 ) );

  menu_handler_inter.insert("Set home", boost::bind(&AddWayPoint::processFeedbackInter, this, _1));
  menu_handler_inter.insert("Attach Points", boost::bind(&AddWayPoint::processFeedbackInter, this, _1));
  menu_handler_inter.insert("Detach Points", boost::bind(&AddWayPoint::processFeedbackInter, this, _1));

  menu_handler_points_inter.insert("Duplicate selected at end in order", boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1));
  menu_handler_points_inter.insert("Duplicate selected at end and reverse", boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1));
  menu_handler_points_inter.insert("Add point here", boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1));
  menu_handler_points_inter.insert("Select all points", boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1));
  menu_handler_points_inter.insert("Deselect all points", boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1));
  menu_handler_points_inter.insert("Recenter", boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1));


  connect(path_generate, SIGNAL(getRobotModelFrame_signal(const std::string, const tf::Transform)), this, SLOT(getRobotModelFrame_slot(const std::string, const tf::Transform)));

  connect(path_generate, SIGNAL(getRobotModelFrame_signal(const std::string, const tf::Transform)), widget_, SLOT(setAddPointUIStartPos(const std::string, const tf::Transform)));

  connect(widget_, SIGNAL(addPoint(tf::Transform)), this, SLOT(addPointFromUI(tf::Transform)));
  connect(widget_, SIGNAL(pointDelUI_signal(std::string)), this, SLOT(pointDeleted(std::string)));
  connect(widget_, SIGNAL(duplicateWaypoint_signal(std::string)), this, SLOT(duplicateWaypoint(std::string)));
  connect(this, SIGNAL(addPointRViz(const tf::Transform &, const int)), widget_, SLOT(insertRow(const tf::Transform &, const int)));
  connect(this, SIGNAL(pointPoseUpdatedRViz(const tf::Transform &, const char *)), widget_, SLOT(pointPosUpdated_slot(const tf::Transform &, const char *)));
  connect(widget_, SIGNAL(pointPosUpdated_signal(const tf::Transform &, const char *)), this, SLOT(pointPoseUpdated(const tf::Transform &, const char *)));
  connect(this, SIGNAL(pointDeleteRviz(int)), widget_, SLOT(removeRow(int)));

  connect(widget_, SIGNAL(cartesianPathParamsFromUI_signal(double, double, double, bool, bool, std::string, bool)), path_generate, SLOT(setCartParams(double, double, double, bool, bool, std::string, bool)));

  connect(path_generate,SIGNAL(wayPointOutOfIK(int,int, std::vector<geometry_msgs::Pose>)),this,SLOT(wayPointOutOfIK_slot(int,int, std::vector<geometry_msgs::Pose>)));
  connect(this,SIGNAL(onUpdatePosCheckIkValidity(const std::vector<tf::Transform>, const int)),path_generate,SLOT(checkWayPointValidity(const std::vector<tf::Transform>, const int)));

  connect(widget_, SIGNAL(parseWayPointBtn_signal()), this, SLOT(parseWayPoints()));
  connect(widget_, SIGNAL(parseWayPointBtnGoto_signal(int, int)), this, SLOT(parseWayPointsGoto(int, int)));
  connect(this, SIGNAL(wayPoints_signal(std::vector<geometry_msgs::Pose>)), path_generate, SLOT(cartesianPathHandler(std::vector<geometry_msgs::Pose>)));
  connect(widget_, SIGNAL(parseConfigBtn_signal(std::vector<double>, bool)), path_generate, SLOT(freespacePathHandler(std::vector<double>, bool)));
  connect(widget_, SIGNAL(configEdited_signal(std::vector<double>)), this, SLOT(cacheConfig(std::vector<double>)));
  connect(widget_, SIGNAL(saveObjectBtn_press(std::string, std::string, int, std::string, peanut_cotyledon::CleanPath, std::string)), this, SLOT(saveWayPointsObject(std::string, std::string, int, std::string, peanut_cotyledon::CleanPath, std::string)));
  connect(widget_, SIGNAL(saveToolBtn_press()), this, SLOT(saveToolPath()));
  connect(widget_, SIGNAL(clearAllPoints_signal()), this, SLOT(clearAllPointsRViz()));
  connect(widget_, SIGNAL(modifyMarkerControl_signal(std::string, geometry_msgs::Pose)), this, SLOT(modifyMarkerControl(std::string, geometry_msgs::Pose)));
  connect(widget_, SIGNAL(transformPointsViz(std::string)), this, SLOT(transformPointsViz(std::string)));
  connect(widget_, SIGNAL(clearAllInteractiveBoxes_signal()), this, SLOT(clearAllInteractiveBoxes()));

  connect(path_generate, SIGNAL(wayPointOutOfIK(int, int)), this, SLOT(wayPointOutOfIK_slot(int, int)));
  connect(this, SIGNAL(onUpdatePosCheckIkValidity(const geometry_msgs::Pose &, const int)), path_generate, SLOT(checkWayPointValidity(const geometry_msgs::Pose &, const int)));

  connect(path_generate, SIGNAL(cartesianPathExecuteStarted()), widget_, SLOT(cartesianPathStartedHandler()));
  connect(path_generate, SIGNAL(cartesianPathExecuteFinished()), widget_, SLOT(cartesianPathFinishedHandler()));

  connect(path_generate, SIGNAL(cartesianPathCompleted(double)), widget_, SLOT(cartPathCompleted_slot(double)));

  connect(path_generate, SIGNAL(sendCartPlanGroup(std::vector<std::string>)), widget_, SLOT(getCartPlanGroup(std::vector<std::string>)));

  connect(widget_, SIGNAL(sendSendSelectedPlanGroup(int)), path_generate, SLOT(getSelectedGroupIndex(int)));

  connect(widget_, SIGNAL(ChangeCheckIK_signal()), path_generate, SLOT(ChangeCheckIk()));

  connect(widget_, SIGNAL(CheckAllPointsIK_signal()), this, SLOT(CheckAllPointsIK()));
  connect(widget_, SIGNAL(RobotIKPlanning_signal(const double, const double, const double, const double)), this, SLOT(RobotIKPlanning(const double, const double, const double, const double)));

  connect(widget_, SIGNAL(ModifyPointsMarkerPose_signal()), this, SLOT(ModifyPointsMarkerPose()));

  connect(this, SIGNAL(initRviz()), path_generate, SLOT(initRvizDone()));
  /*!  With the signal initRviz() we call a function GenerateCartesianPath::initRvizDone() which sets the initial parameters of the MoveIt enviroment.

    */
  Q_EMIT initRviz();
  ROS_INFO("ready.");

  tfListener = new tf2_ros::TransformListener(tfBuffer);
}

void AddWayPoint::load(const rviz::Config &config)
{
  /*! \brief Setting up the configurations for the Panel of the RViz enviroment.
   */
  rviz::Panel::load(config);
  QString text_entry;
  ROS_INFO_STREAM("rviz: Initializing the user interaction planning panel");
  if (config.mapGetString("TextEntry", &text_entry))
  {
    ROS_INFO_STREAM("Loaded TextEntry with value: " << text_entry.toStdString());
  }

  ROS_INFO_STREAM("rviz Initialization Finished reading config file");
}

void AddWayPoint::save(rviz::Config config) const
{
  /// Allowing the user to save the current configuration of the panel
  ROS_INFO_STREAM("Saving configuration");
  rviz::Panel::save(config);
  config.mapSetValue("TextEntry", QString::fromStdString(std::string("test_field")));
}

void AddWayPoint::addPointFromUI(const tf::Transform point_pos)
{
  /*! Function for handling the signal of the RQT to add a new Way-Point in the RViz enviroment.
  */
  makeArrow(point_pos, count);
  server->applyChanges();
}

void AddWayPoint::cacheConfig(std::vector<double> config)
{
  AddWayPoint::config = config;
}

void AddWayPoint::processFeedbackInter(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      //get the menu item which is pressed
      interactive_markers::MenuHandler::EntryHandle menu_item = feedback->menu_entry_id;
      std::string marker_name = feedback->marker_name;
      InteractiveMarker interaction_marker;
      if (!server->get(feedback->marker_name.c_str(), interaction_marker)){
        ROS_ERROR_STREAM("Could not get marker with ID: "<<feedback->marker_name.c_str());
        return;
      }

      if (menu_item == 1)
      {
        // Cache home position
        parent_home_ = feedback->pose;
      }
      else if (menu_item == 2){
        ROS_INFO("Attaching points to object");
        points_attached_to_object = true;
      }
      else if (menu_item == 3){
        ROS_INFO("Detaching points from object");
        points_attached_to_object = false;
      }
      break;
    }
    
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    { 
      if (points_attached_to_object){
        // Get markers 
        std::vector<InteractiveMarker> markers;
        InteractiveMarker cur_marker;
        for (int i = 1; i <= waypoints_pos.size(); i++)
        {
          if (!server->get(std::to_string(i), cur_marker)){
            ROS_ERROR_STREAM("Could not get marker with ID: "<<i);
            return;
          }
          markers.push_back(cur_marker);
        }

        tf::Pose T_o2_w, T_o1_w, T_o1_w_inv;
        geometry_msgs::Pose T_o2_w_msg, T_o1_w_msg;

        T_o2_w_msg = feedback->pose; // Transformation of frame O2 wrt world
        T_o1_w_msg = parent_home_; // Transformation of frame O1 wrt world
        tf::poseMsgToTF(T_o2_w_msg, T_o2_w);
        tf::poseMsgToTF(T_o1_w_msg, T_o1_w);
        T_o1_w_inv = T_o1_w.inverse();

        // Apply delta to all markers    
        tf::Pose p1, p2;
        geometry_msgs::Pose current_marker_pose_msg;
        for (InteractiveMarker cur_marker : markers)
        {
          current_marker_pose_msg =  cur_marker.pose;
          tf::poseMsgToTF(current_marker_pose_msg, p1);

          // Apply transform
          p2 = T_o2_w * T_o1_w_inv * p1;

          pointPoseUpdated(p2, cur_marker.name.c_str());
          Q_EMIT pointPoseUpdatedRViz(p2, cur_marker.name.c_str());
        }
      }

      // Update home markers
      parent_home_ = feedback->pose;
      tf::poseMsgToTF(feedback->pose, box_pos);

    }
  }
}

void AddWayPoint::ModifyPointsMarkerPose(){
  
  InteractiveMarker interaction_marker;
  InteractiveMarkerControl control_button;

  // Get markers
  if (!server->get("move_points_button", interaction_marker)){
    ROS_ERROR("Could not get marker move_points_button");
    return;
  }
  if (interaction_marker.controls.size() == 0){
    ROS_ERROR("Points marker does not have control");
    return;
  }
  control_button = interaction_marker.controls.at(0);
  
  // Check for nan 
  // if(std::isnan(parent_home_.position.x) || std::isnan(parent_home_.position.y) || std::isnan(parent_home_.position.z) ||
  //    std::isnan(parent_home_.orientation.x) || std::isnan(parent_home_.orientation.y) || std::isnan(parent_home_.orientation.z) ||
  //    std::isnan(parent_home_.orientation.w)){
  //      ROS_ERROR("parent_home_ has an invalid pose");
  //      return;
  //   }
  // if(std::isinf(parent_home_.position.x) || std::isinf(parent_home_.position.y) || std::isinf(parent_home_.position.z) ||
  //    std::isinf(parent_home_.orientation.x) || std::isinf(parent_home_.orientation.y) || std::isinf(parent_home_.orientation.z) ||
  //    std::isinf(parent_home_.orientation.w)){
  //      ROS_ERROR("parent_home_ has an invalid pose: infinity");
  //      return;
  //   }
    
    // Update marker pose 
  addPoseOffset(parent_home_, interaction_marker.pose);
  points_parent_home_ = interaction_marker.pose;

  // Update control button
  if (control_button.markers.size() == 0){
    ROS_ERROR("Points marker control button does not have marker");
    return;
  }
  control_button.markers.at(0) = makeInterArrow(interaction_marker, 1);

  // Update server
  interaction_marker.controls.at(0) = control_button;
  server->insert(interaction_marker);
  if (!server->setCallback(interaction_marker.name, boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1))){
    ROS_ERROR("Could not insert marker");
  }
  server->applyChanges();

}

void AddWayPoint::processFeedbackPointsInter(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  
  switch (feedback -> event_type){
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      interactive_markers::MenuHandler::EntryHandle menu_item = feedback->menu_entry_id;
      std::string marker_name = feedback->marker_name;

      if (menu_item == 1)
        { 
          // Duplicate all selected in place
          // (1) find all selected markers
          std::vector<tf::Transform> selected;
          InteractiveMarker cur_marker;
          int last_marker_index = 1;
          for (int i = 1; i <= waypoints_pos.size(); i++)
          {
            if (!server->get(std::to_string(i), cur_marker)){
              ROS_ERROR_STREAM("Could not get marker with ID: "<<i);
              return;
            }
            if (cur_marker.controls.size() > 2)
            {
              geometry_msgs::Pose cur_pos = cur_marker.pose;
              tf::Transform point_pos;
              cur_pos.position.z += 0.01; // move it up a centimeter in z because no two points can be in identically the same spot
              tf::poseMsgToTF(cur_pos, point_pos);
              selected.push_back(point_pos);
              last_marker_index = stoi(cur_marker.name);
            }
          }

          std::vector<std::string> duplicated_names;
          // (2) Duplicate in place
          std::vector<tf::Transform>::iterator insertion_point = waypoints_pos.begin();
          advance(insertion_point, last_marker_index);
          insert(insertion_point, selected);
          // (4) set selected to the duplicated points
          for (int i = last_marker_index + 1; i <= last_marker_index + selected.size(); i++)
          {
            changeMarkerControlAndPose(std::to_string(i), "adjust_frame");
          }

          // Update marker
          modifyMarkerControl(mesh_name_, parent_home_);
        }
        else if (menu_item == 2)
        { // duplicate and flip
          // (1) find all selected markers
          std::vector<tf::Transform> selected;
          InteractiveMarker cur_marker;
          int last_marker_index = 1;
          for (int i = 1; i <= waypoints_pos.size(); i++)
          {
            if (!server->get(std::to_string(i), cur_marker)){
              ROS_ERROR_STREAM("Could not get marker with ID: "<<i);
              return;
            }
            if (cur_marker.controls.size() > 2)
            {
              geometry_msgs::Pose cur_pos = cur_marker.pose;
              tf::Transform point_pos;
              cur_pos.position.z += 0.01; // move it up a centimeter in z because no two points can be in identically the same spot
              tf::poseMsgToTF(cur_pos, point_pos);
              selected.push_back(point_pos);
              last_marker_index = stoi(cur_marker.name);
            }
          }

          std::reverse(selected.begin(), selected.end()); // This is the only different line

          // (2) Duplicate in place
          std::vector<tf::Transform>::iterator insertion_point = waypoints_pos.begin();
          advance(insertion_point, last_marker_index);
          insert(insertion_point, selected);
          // (4) set selected to the duplicated points
          for (int i = last_marker_index + 1; i <= last_marker_index + selected.size(); i++)
          {
            changeMarkerControlAndPose(std::to_string(i), "adjust_frame");
          }
          // Update marker
          modifyMarkerControl(mesh_name_, parent_home_);
        }
        else if (menu_item == 3)
        {
          std::vector<tf::Transform>::iterator insertion_point = waypoints_pos.end();
          tf::Transform point_pos;
          geometry_msgs::Pose cur_pos = feedback->pose;
          tf::poseMsgToTF(cur_pos, point_pos);
          std::vector<tf::Transform> pos_vec = {point_pos};
          insert(insertion_point, pos_vec);
          
          // Update marker
          modifyMarkerControl(mesh_name_, parent_home_);
        } 
        else if (menu_item == 4){
          // Select all points
          InteractiveMarker cur_marker;
          ROS_DEBUG("Selecting all points");
          for (int i = 1; i <= waypoints_pos.size(); i++)
          {
            if (!server->get(std::to_string(i), cur_marker)){
              ROS_ERROR_STREAM("Could not get marker with ID: "<<i);
              return;
            }
            changeMarkerControlAndPose(cur_marker.name.c_str(), "adjust_frame");
            server->applyChanges();
          }
        }
        else if (menu_item == 5){
          // Deselect all points
          ROS_DEBUG("Deselecting all points");
          for (int i = 1; i <= waypoints_pos.size(); i++)
          {
            changeMarkerControlAndPose(std::to_string(i), "adjust_hide");
            server->applyChanges();
          }
        }
        else if (menu_item == 6){
          ModifyPointsMarkerPose();
        }
        else{
          ROS_ERROR("Menu button not implemented");
          break;
        }
    }
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    { 
        // Get markers 
        std::vector<InteractiveMarker> markers;
        InteractiveMarker cur_marker;
        for (int i = 1; i <= waypoints_pos.size(); i++)
        {
          if (!server->get(std::to_string(i), cur_marker)){
            ROS_ERROR_STREAM("Could not get marker with ID: "<<i);
            return;
          }
          if (cur_marker.controls.size() > 2){
            // Only move selected points
            markers.push_back(cur_marker);
          }
        }

        tf::Pose T_o2_w, T_o1_w, T_o1_w_inv, T;
        geometry_msgs::Pose T_o2_w_msg, T_o1_w_msg;

        T_o2_w_msg = feedback->pose; // Transformation of frame O2 wrt world
        T_o1_w_msg = points_parent_home_; // Transformation of frame O1 wrt world
        tf::poseMsgToTF(T_o2_w_msg, T_o2_w);
        tf::poseMsgToTF(T_o1_w_msg, T_o1_w);
        T_o1_w_inv = T_o1_w.inverse();
        T =  T_o2_w * T_o1_w_inv; // Final transformation matrix

        // Apply delta to all markers    
        tf::Pose p1, p2;
        geometry_msgs::Pose current_marker_pose_msg;
        for (InteractiveMarker cur_marker : markers)
        {
          current_marker_pose_msg =  cur_marker.pose;
          tf::poseMsgToTF(current_marker_pose_msg, p1);

          // Apply transform
          p2 = T * p1;

          pointPoseUpdated(p2, cur_marker.name.c_str());
          Q_EMIT pointPoseUpdatedRViz(p2, cur_marker.name.c_str());
        }

      // Save pose
      points_parent_home_ = feedback -> pose;
    }
  }
}

void AddWayPoint::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  /*! This function is one of the most essential ones since it handles the events of the InteractiveMarkers from the User in the RViz Enviroment.
      In this function we have handlers for all the necessary events of the User Interaction with the InteractiveMarkers.
      When the user clicks on the User Interactive Arrow, it acts as button and adds new Way-Point in the RViz enviroment.
      When the User changes the pose of an InteractiveMarker from the RViz enviroment the position of the InteractiveMarker is updated synchronously in the RViz enviroment and in the RQT Widget.
      The Menu handlers take care of the User selected items from the menu of the Way-Point and call the necessary functions to change their state depending on the item selected from the menu.
  */
  switch (feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
  {

    tf::Transform point_pos;
    tf::poseMsgToTF(feedback->pose, point_pos);
    pointPoseUpdated(point_pos, feedback->marker_name.c_str());

    Q_EMIT pointPoseUpdatedRViz(point_pos, feedback->marker_name.c_str());

    break;
  }
  case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
  {
    //get the menu item which is pressed
    interactive_markers::MenuHandler::EntryHandle menu_item = feedback->menu_entry_id;
    interactive_markers::MenuHandler::CheckState state;
    std::string marker_name = feedback->marker_name;

    if (menu_item == 1)
    {
      int marker_nr = atoi(marker_name.c_str());
      Q_EMIT pointDeleteRviz(marker_nr);
      pointDeleted(marker_name);
    }
    else if (menu_item == 2)
    {
      ROS_INFO_STREAM("The selected marker:" << feedback->marker_name.c_str() << "is shown with in frame fine adjustment");
      std::string control_mode = "adjust_frame";
      changeMarkerControlAndPose(feedback->marker_name.c_str(), control_mode);
    }
    else if (menu_item == 3)
    {
      std::string control_mode = "adjust_eef";
      ROS_INFO_STREAM("The selected marker:" << feedback->marker_name.c_str() << "is shown with in eef fine adjustment");
      changeMarkerControlAndPose(feedback->marker_name.c_str(), control_mode);
    }
    else if (menu_item == 4)
    {
      std::string control_mode = "adjust_hide";
      ROS_INFO_STREAM("Turning off fine adjustment");
      changeMarkerControlAndPose(feedback->marker_name.c_str(), control_mode);
    }
    else if (menu_item == 5)
    {
      std::vector<tf::Transform>::iterator insertion_point = waypoints_pos.begin();
      int marker_index = stoi(feedback->marker_name);
      advance(insertion_point, marker_index);

      tf::Transform point_pos;
      geometry_msgs::Pose cur_pos = feedback->pose;
      cur_pos.position.z += 0.01; // move it up a centimeter in z because points cant be identical
      tf::poseMsgToTF(cur_pos, point_pos);
      std::vector<tf::Transform> pos_vec = {point_pos};
      insert(insertion_point, pos_vec);
    }
    else
    {
      ROS_ERROR_STREAM("an unknown menu_item selection was caught in feedback, the menu_item is: " << std::to_string(menu_item));
    }
    break;
  }
  }
  server->applyChanges();
}

void AddWayPoint::pointPoseUpdated(const tf::Transform &point_pos, const char *marker_name)
{
  /*!
      @param point_pos takes the changed position of the InteractiveMarker either from RViz enviroment or the RQT.The vector for storing the Way-Points and the User Interaction Marker are updated according to the value of this parameter.
      @param marker_name is passed from this function either by taking information of the name of the Marker that has its position changed or by the RQT enviroment.

      Depending on the name of the Marker we either update the pose of the User Interactive Arrow or the Way-Point that is selected either from the RViz enviroment or the RQT Widget.
      In the case of updating a pose of a Way-Point, the corresponding position of the vector that stores all the poses for the Way-Points is updated as well.
  */
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(point_pos, pose);
  std::stringstream s;

  if (strcmp("add_point_button", marker_name) == 0)
  {

    box_pos = point_pos;
    s << "add_point_button";
  }
  else
  {
    int index = atoi(marker_name);

    if ((index > waypoints_pos.size()) || (index < 1) )
    {
      ROS_ERROR_STREAM("Trying to access incorrect index: "<< index);
      return;
    }

    waypoints_pos[index - 1] = point_pos;

    s << index;
    Q_EMIT onUpdatePosCheckIkValidity(waypoints_pos, index);
  }

  server->setPose(s.str(), pose);
  server->applyChanges();
}

Marker AddWayPoint::makeWayPoint(InteractiveMarker &msg)
{
  /*! Define a type and properties of a Marker for the Way-Point.
      This will be use as a base to define the shape, color and scale of the InteractiveMarker for the Way-Points.
  */
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale = WAY_POINT_SCALE_CONTROL;

  marker.color = WAY_POINT_COLOR;
  marker.action = visualization_msgs::Marker::ADD;
  return marker;
}

void AddWayPoint::makeArrowControlDefault(InteractiveMarker &msg)
{
  InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;

  control_menu.interaction_mode = InteractiveMarkerControl::MENU;

  control_menu.name = "menu_select";
  control_menu.markers.push_back(makeWayPoint(msg));
  msg.controls.push_back(control_menu);
}

void AddWayPoint::makeArrowControlDetails(InteractiveMarker &msg, bool is_fixed_frame)
{

  InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;

  control_menu.interaction_mode = InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);

  InteractiveMarkerControl control_view_details;
  control_view_details.always_visible = true;
  if (is_fixed_frame)
  {
    control_view_details.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  double q_norm = sqrt(2);

  //*************rotate and move around the x-axis********************
  control_view_details.orientation.w = 1/q_norm;
  control_view_details.orientation.x = 1/q_norm;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 0;

  control_view_details.name = "rotate_x";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_view_details);

  control_view_details.name = "move_x";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);
  //*****************************************************************

  //*************rotate and move around the z-axis********************
  control_view_details.orientation.w = 1/q_norm;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 1/q_norm;
  control_view_details.orientation.z = 0;

  control_view_details.name = "rotate_z";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_view_details);

  control_view_details.name = "move_z";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);
  //*****************************************************************

  //*************rotate and move around the y-axis********************
  control_view_details.orientation.w = 1/q_norm;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 1/q_norm;

  control_view_details.name = "rotate_y";
  control_view_details.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  msg.controls.push_back(control_view_details);

  control_view_details.name = "move_y";
  control_view_details.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);

  msg.controls.push_back(control_view_details);
  InteractiveMarkerControl arrow_viz_control;
  arrow_viz_control.always_visible = true;

  arrow_viz_control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;
  arrow_viz_control.name = "arrow";
  arrow_viz_control.markers.push_back(makeWayPoint(msg));
  msg.controls.push_back(arrow_viz_control);
  //*****************************************************************
}

void AddWayPoint::makeArrow(const tf::Transform &point_pos, int count_arrow) //
{
  /*! Function for adding a new Way-Point in the RViz scene and here we send the signal to notify the RQT Widget that a new Way-Point has been added.
        */
  InteractiveMarker int_marker;
  ROS_DEBUG_STREAM("Adding point! " << std::to_string(count_arrow));
  ROS_DEBUG_STREAM("Markers intractive frame is: " << target_frame_);

  int_marker.header.frame_id = target_frame_;

  ROS_DEBUG_STREAM("Markers has frame id: " << int_marker.header.frame_id);

  int_marker.scale = INTERACTIVE_MARKER_SCALE;

  tf::poseTFToMsg(point_pos, int_marker.pose);

  std::vector<tf::Transform>::iterator it_pos = std::find((waypoints_pos.begin()), (waypoints_pos.end() - 1), point_pos);

  /*! Check the positions and orientations vector if they are emtpy. If it is empty we have our first Way-Point.
        */
  if (waypoints_pos.empty())
  {
    ROS_DEBUG("Adding first arrow!");
    count_arrow++;
    count = count_arrow;

    waypoints_pos.push_back(point_pos);
    Q_EMIT addPointRViz(point_pos, count);
  }
  /*! Check if we have points in the same position in the scene. If we do, do not add one and notify the RQT Widget so it can also add it to the TreeView.
        */
  else if ((it_pos == (waypoints_pos.end())) || (point_pos.getOrigin() != waypoints_pos.at(count_arrow - 1).getOrigin())) // && (point_pos.getOrigin() != waypoints_pos.at(count_arrow-1).getOrigin()) //(it_pos == waypoints_pos.end()) &&
  {
    count_arrow++;
    count = count_arrow;

    waypoints_pos.push_back(point_pos);

      ROS_DEBUG_STREAM("Adding new arrow! with point_pos " << int_marker.pose);
      Q_EMIT addPointRViz(point_pos,count);
    }
    else
    {
      //if we have arrow, ignore adding new one and inform the user that there is arrow (waypoint at that location)
      ROS_INFO("There is already a arrow at that location, can't add new one!!");
    }
/*******************************************************************************************************************************************************************************************************************/
    std::stringstream s;
    s << count_arrow;
    ROS_DEBUG("end of make arrow, count is:%d, positions count:%ld",count,waypoints_pos.size());
    int_marker.name = s.str();
    int_marker.description = s.str();

    makeArrowControlDefault(int_marker);
    server->insert( int_marker);
    server->setCallback( int_marker.name, boost::bind( &AddWayPoint::processFeedback, this, _1 ));
    menu_handler.apply(*server,int_marker.name);
    //server->applyChanges();
    Q_EMIT onUpdatePosCheckIkValidity(waypoints_pos, count_arrow);
}

void AddWayPoint::clearAllInteractiveBoxes()
{
  /*! Function for clearing all the boxes from the scene, change everything back to a standard arrow
    */
  std::string default_string = "adjust_hide";
  for (int i = 1; i <= waypoints_pos.size(); i++)
  {
    ROS_DEBUG_STREAM("clearing box for " << std::to_string(i));
    changeMarkerControlAndPose(std::to_string(i), default_string);
    server->applyChanges();
  }
}

void AddWayPoint::changeMarkerControlAndPose(std::string marker_name, std::string control_mode)
{

  /*! Handling the events from the clicked Menu Items for the Control of the Way-Point.
       Here the user can change the control either to freely move the Way-Point or get the 6DOF pose control option.
   */
  InteractiveMarker int_marker;
  if (!server->get(marker_name, int_marker)){
    ROS_ERROR_STREAM("Could not get marker with ID: "<<marker_name);
    return;
  }
  int_marker.controls.clear();

  if (control_mode == "adjust_eef")
  {
    bool fixed_frame = false;
    makeArrowControlDetails(int_marker, fixed_frame);
  }
  else if (control_mode == "adjust_frame")
  {
    bool fixed_frame = true;
    makeArrowControlDetails(int_marker, fixed_frame);
  }
  else if (control_mode == "adjust_hide")
  {
    makeArrowControlDefault(int_marker);
  }
  else
  {
    ROS_ERROR_STREAM("unknown control mode :" << control_mode);
  }
    server->insert( int_marker);
    menu_handler.apply(*server,int_marker.name);
    server->setCallback( int_marker.name, boost::bind( &AddWayPoint::processFeedback, this, _1 ));
    Q_EMIT onUpdatePosCheckIkValidity(waypoints_pos, atoi(marker_name.c_str()));
}

void AddWayPoint::pointDeleted(std::string marker_name)
{
  /*! The point can be deleted either from the RViz or the RQT Widged.
       This function handles the event of removing a point from the RViz enviroment. It finds the name of the selected marker which the user wants to delete and updates the RViz enviroment and the vector that contains all the Way-Points.
    */
  for (int i = 0; i < waypoints_pos.size(); i++)
    ROS_DEBUG_STREAM("vecotr before delete: \n"
                     << "x:" << waypoints_pos[i].getOrigin().x() << "; " << waypoints_pos[i].getOrigin().y() << "; " << waypoints_pos[i].getOrigin().z() << ";\n");

  //get the index of the selected marker
  int index = atoi(marker_name.c_str());
  server->erase(marker_name.c_str());
  waypoints_pos.erase(waypoints_pos.begin() + index - 1);

  for (int i = 0; i < waypoints_pos.size(); i++)
    ROS_DEBUG_STREAM("vecotr before delete: \n"
                     << "x:" << waypoints_pos[i].getOrigin().x() << "; " << waypoints_pos[i].getOrigin().y() << "; " << waypoints_pos[i].getOrigin().z() << ";\n");
  //InteractiveMarker int_marker;
  for (int i = index + 1; i <= count; i++)
  {
    std::stringstream s;
    s << i;
    server->erase(s.str());
    makeArrow(waypoints_pos[i - 2], (i - 1));
  }
  count--;
  server->applyChanges();
}
void AddWayPoint::insert(std::vector<tf::Transform>::iterator insert_it, std::vector<tf::Transform> pose_vector)
{
  waypoints_pos.insert(insert_it, pose_vector.begin(), pose_vector.end());

  std::vector<tf::Transform> waypoints_pos_copy(waypoints_pos);

  clearAllPointsRViz();

  std::vector<tf::Transform>::iterator point_pos = waypoints_pos_copy.begin();
  int i = 0;
  for (point_pos = waypoints_pos_copy.begin(); point_pos < waypoints_pos_copy.end(); point_pos++)
  {
    makeArrow(*point_pos, i);
    i++;
  }

  waypoints_pos = waypoints_pos_copy;
  server->applyChanges();
}

Marker AddWayPoint::makeMeshResourceMarker(std::string mesh_name, geometry_msgs::Pose object_pose){
  // Define the Marker Mesh which the user can add new Way-Points with
  Marker marker;

  marker.type = Marker::MESH_RESOURCE;
  marker.scale = MESH_SCALE_CONTROL;
  marker.mesh_resource = "package://peanut_datasets_pkg/meshes/" + mesh_name; 

  marker.pose = CONTROL_MARKER_POSE;
  //make the markers with interesting color
  marker.color = ARROW_INTER_COLOR;

  return marker;

}

Marker AddWayPoint::makeInterArrow(InteractiveMarker &msg, const int type)
{
  /*! Define the Marker Arrow which the user can add new Way-Points with.

   */
  //define a marker
  Marker marker;

  if (type == 0){
  marker.type = Marker::CUBE;
  }
  else{
    marker.type = Marker::SPHERE;
  }
  marker.scale = ARROW_INTER_SCALE_CONTROL;

  //make the markers with interesting color
  marker.color = ARROW_INTER_COLOR;

  return marker;
}

InteractiveMarkerControl &AddWayPoint::makeInteractiveMarkerControl(InteractiveMarker &msg, const int type)
{
  /*! Set the User Interactive Marker with 6DOF control.
  */
  // //control for button interaction
  InteractiveMarkerControl control_button;
  control_button.always_visible = true;
  control_button.interaction_mode = InteractiveMarkerControl::BUTTON;
  control_button.name = "button_interaction";
  control_button.markers.push_back(makeInterArrow(msg, type));

  msg.controls.push_back(control_button);
  //server.reset( new interactive_markers::InteractiveMarkerServer("moveit_cartesian_plan_plugin","",false));
  InteractiveMarkerControl control_inter_arrow;
  control_inter_arrow.always_visible = true;

  double q_norm = sqrt(2);

  //*************rotate and move around the x-axis********************
  control_inter_arrow.orientation.w = 1/q_norm;
  control_inter_arrow.orientation.x = 1/q_norm;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 0;

  control_inter_arrow.name = "rotate_x";
  control_inter_arrow.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  control_inter_arrow.name = "move_x";
  control_inter_arrow.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************

  //*************rotate and move around the z-axis********************
  control_inter_arrow.orientation.w = 1/q_norm;
  control_inter_arrow.orientation.x = 0;
  control_inter_arrow.orientation.y = 1/q_norm;
  control_inter_arrow.orientation.z = 0;

  control_inter_arrow.name = "rotate_z";
  control_inter_arrow.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  control_inter_arrow.name = "move_z";
  control_inter_arrow.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************

  //*************rotate and move around the y-axis********************
  control_inter_arrow.orientation.w = 1/q_norm;
  control_inter_arrow.orientation.x = 0;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 1/q_norm;

  control_inter_arrow.name = "rotate_y";
  control_inter_arrow.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  control_inter_arrow.name = "move_y";
  control_inter_arrow.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************
  InteractiveMarkerControl arrow_marker;
  arrow_marker.always_visible = true;

  // arrow_marker.markers.push_back( makeInterArrow(msg) );
  msg.controls.push_back(arrow_marker);

  return msg.controls.back();
}

void AddWayPoint::makeInteractiveMarker()
{
  /*! Create the User Interactive Marker and update the RViz enviroment.
   */
  InteractiveMarker inter_arrow_marker_;
  inter_arrow_marker_.header.frame_id = target_frame_;
  inter_arrow_marker_.scale = ARROW_INTERACTIVE_SCALE;

  ROS_DEBUG_STREAM("Marker Frame is: " << target_frame_);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(box_pos, inter_arrow_marker_.pose);

  inter_arrow_marker_.description = "Interaction Marker";

  //button like interactive marker. Detect when we have left click with the mouse and add new arrow then
  inter_arrow_marker_.name = "add_point_button";

  makeInteractiveMarkerControl(inter_arrow_marker_);
  server->insert(inter_arrow_marker_);
  menu_handler_inter.apply(*server, inter_arrow_marker_.name);
  //add interaction feedback to the markers
  server->setCallback(inter_arrow_marker_.name, boost::bind(&AddWayPoint::processFeedbackInter, this, _1));
}

void AddWayPoint::makePointsInteractiveMarker()
{
  //Create the Points Interactive Marker and update the RViz enviroment.
  
  InteractiveMarker inter_arrow_marker_;
  inter_arrow_marker_.header.frame_id = target_frame_;
  inter_arrow_marker_.scale = ARROW_INTERACTIVE_SCALE;

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(box_pos, inter_arrow_marker_.pose);

  // Offset pose
  inter_arrow_marker_.pose.position.x += 0.2;
  inter_arrow_marker_.pose.position.y += 0.2;
  inter_arrow_marker_.pose.position.z -= 0.2;
  inter_arrow_marker_.description = "Interaction Marker";

  //button like interactive marker. Detect when we have left click with the mouse and add new arrow then
  inter_arrow_marker_.name = "move_points_button";

  makeInteractiveMarkerControl(inter_arrow_marker_, 1);
  server->insert(inter_arrow_marker_);
  menu_handler_points_inter.apply(*server, inter_arrow_marker_.name);
 
  server->setCallback(inter_arrow_marker_.name, boost::bind(&AddWayPoint::processFeedbackPointsInter, this, _1));
}


void AddWayPoint::parseWayPoints()
{
  /*! Get the vector of all Way-Points and convert it to geometry_msgs::Pose and send Qt signal when ready.
   */
  geometry_msgs::Pose target_pose;
  std::vector<geometry_msgs::Pose> waypoints;

  for (int i = 0; i < waypoints_pos.size(); i++)
  {

    tf::poseTFToMsg(waypoints_pos[i], target_pose);

    waypoints.push_back(target_pose);
  }

  Q_EMIT wayPoints_signal(waypoints);
}
void AddWayPoint::parseWayPointsGoto(int min_index, int max_index)
{
  geometry_msgs::Pose target_pose;
  std::vector<geometry_msgs::Pose> waypoints;

  try
  {
    for (int i = min_index; i < max_index; i++)
    {
      tf::poseTFToMsg(waypoints_pos[i], target_pose);
      waypoints.push_back(target_pose);
    }
  }
  catch (...)
  {
    ROS_ERROR("Unknown error when slicing points to go to a specific point, check your min and max indicies");
  }
  ROS_INFO_STREAM("Playing subset of waypoints from start index (inclusive, zero indexed)" << std::to_string(min_index) << " to ending index (exclusive)" << std::to_string(max_index));
  Q_EMIT wayPoints_signal(waypoints);
}
void AddWayPoint::saveToolPath(){
  /*! Function for saving all the Way-Points into yaml file.
        This function opens a Qt Dialog where the user can set the name of the Way-Points file and the location.
        Furthermore, it parses the way-points into a format that could be also loaded into the Plugin.
    */
    ROS_INFO("Saving tool path");
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Save Way Points"), ".yaml",
        tr("Way Points (*.yaml);;All Files (*)"));

    if (fileName.isEmpty())
      return;
    else{
          QFile file(fileName);
          if (!file.open(QIODevice::WriteOnly)) {
              QMessageBox::information(this, tr("Unable to open file"),
                  file.errorString());
                  file.close();
          return;
        }

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "frame_id";
    out << YAML::Value << target_frame_;

    //todo save the config here.
    out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;

    for(int i=0;i<waypoints_pos.size();i++)
    {
      out << YAML::BeginMap;

      out << YAML::Key << "position";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << waypoints_pos[i].getOrigin().x();
      out << YAML::Key << "y" << YAML::Value << waypoints_pos[i].getOrigin().y();
      out << YAML::Key << "z" << YAML::Value << waypoints_pos[i].getOrigin().z();
      out << YAML::EndMap;

      out << YAML::Key << "orientation";
      out << YAML::Value;

      tf::Quaternion q;
      tf::Matrix3x3 m(waypoints_pos[i].getRotation());
      m.getRotation(q);
      out << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << q.x();
      out << YAML::Key << "y" << YAML::Value << q.y();
      out << YAML::Key << "z" << YAML::Value << q.z();
      out << YAML::Key << "w" << YAML::Value << q.w();
      out << YAML::EndMap;

      out << YAML::EndMap;

    }
    out << YAML::EndSeq; // End list of points
    out << YAML::Key << "start_config" << YAML::Value << YAML::BeginSeq;
    out << config.at(0) << config.at(1) << config.at(2) << config.at(3) << config.at(4) << config.at(5) << config.at(6);
    out << YAML::EndSeq;// and configuration list
    out << YAML::EndMap;

    std::ofstream myfile;
    myfile.open (fileName.toStdString().c_str());
    myfile << out.c_str();
    myfile.close();

  }
}

bool AddWayPoint::getObjectWithID(std::string floor_name, std::string area_name, int object_id, peanut_cotyledon::Object& desired_obj){
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

void AddWayPoint::saveWayPointsObject(std::string floor_name, std::string area_name, int object_id, std::string task_name, peanut_cotyledon::CleanPath clean_path, std::string mesh_name)
{
  /*! Function for saving all the Way-Points into yaml file.
        This function opens a Qt Dialog where the user can set the name of the Way-Points file and the location.
        Furthermore, it parses the way-points into a format that could be also loaded into the Plugin.
  */
  ROS_INFO("Saving clean path");
  ROS_INFO_STREAM("The frame the poses are being saved in is " << target_frame_);
  
  // Transforms and poses
  geometry_msgs::Transform target_map_tfmsg, object_world_tfmsg;
  tf::Transform target_map_tf, object_world_tf, target_object_tf;
  std::vector<geometry_msgs::Pose> waypoints_map_frame, waypoints_object_frame;
  Eigen::Affine3d object_world_eigen;

  // Used for temp storing transforms
  tf::Transform waypoint_tf;
  geometry_msgs::Pose waypoint_pose;

  // Empty cached path and joint trajectory
  std::vector<peanut_cotyledon::CachedPath> one_cached_path_vec;
  trajectory_msgs::JointTrajectory empty_joint_traj;
  peanut_cotyledon::CachedPath one_cached_path;
  one_cached_path_vec.push_back(one_cached_path);
  
  // Objects
  peanut_cotyledon::Object desired_object;

  // Get tf transforms
  try
  {
    target_map_tfmsg = tfBuffer.lookupTransform("map", target_frame_, ros::Time(0)).transform;
    tf::transformMsgToTF(target_map_tfmsg, target_map_tf);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Unable to save because not able to transform");
    return;
  }

  // Get object transform
  if (!getObjectWithID(floor_name, area_name, object_id, desired_object)){
    ROS_ERROR_STREAM("Could not find object with ID"<<object_id);
    return;
  }

  // Update object info
  // Update mesh name
  mesh_name_ = mesh_name;
  if(desired_object.geometry_path.size() == 0){
    desired_object.geometry_path.push_back(mesh_name);  
  }
  else{
    desired_object.geometry_path.at(0) = mesh_name;
  }

  // Assuming that the object pose has remained constant
  desired_object.origin.translation.x = parent_home_.position.x;
  desired_object.origin.translation.y = parent_home_.position.y;
  desired_object.origin.translation.z = parent_home_.position.z;
  desired_object.origin.rotation = parent_home_.orientation;

  object_world_tfmsg = desired_object.origin;
  tf::transformMsgToTF(object_world_tfmsg, object_world_tf);
  target_object_tf = object_world_tf.inverse() * target_map_tf;

  // Transform points    
  ROS_INFO_STREAM("Saving "<<waypoints_pos.size()<< " points");
  for (auto const waypoint_pos_i : waypoints_pos)
  { 
    // Poses are in map frame
    waypoint_tf = target_map_tf * waypoint_pos_i;
    tf::poseTFToMsg (waypoint_tf, waypoint_pose);
    waypoints_map_frame.push_back(waypoint_pose);

    // Poses in object frame
    waypoint_tf = target_object_tf * waypoint_pos_i;
    tf::poseTFToMsg (waypoint_tf, waypoint_pose);
    waypoints_object_frame.push_back(waypoint_pose); 
  }

  /* 
  Save cached path and robot poses in clean path
  Cached path poses are in map frame
  Clean path poses are in object frame
  When saving, cached paths are intialized to empty joint trajectories
  */
  if (clean_path.cached_paths.empty()){
    clean_path.cached_paths = one_cached_path_vec;
  }
  clean_path.object_poses = waypoints_object_frame;
  clean_path.cached_paths.at(0).robot_poses = waypoints_map_frame;
  clean_path.cached_paths.at(0).cached_path = empty_joint_traj;

  // Set clean path
  peanut_cotyledon::SetCleanPath srv;
  srv.request.floor_name = floor_name;
  srv.request.area_name = area_name;
  srv.request.object_id = object_id;
  srv.request.task_name = task_name;
  srv.request.clean_path = clean_path;

  if(set_clean_path_proxy_.call(srv))
  {
    if(srv.response.success){
      ROS_INFO("Successfully saved");
    }
    else {
      ROS_ERROR_STREAM("clean path floor " << floor_name << " area " << area_name << " object_id " << std::to_string(object_id) << "task_name " << task_name << " not able to set");
      return;
    }
  }
  else
  {
    ROS_ERROR_STREAM("clean path floor " << floor_name << " area " << area_name << " object_id " << std::to_string(object_id) << "task_name " << task_name << " not able to set");
    return;
  }

  // Set objects 
  peanut_cotyledon::SetObjects set_srv;
  set_srv.request.floor_name = floor_name;
  set_srv.request.area_name = area_name;
  set_srv.request.objects.push_back(desired_object);

  if (set_objects_proxy_.call(set_srv)){
    if (!set_srv.response.success){
      ROS_ERROR("Could not set object");
    }
  }
  else{
    ROS_ERROR("Could not call set objects service");
    return;
  }

  ROS_INFO("Saving clean path finished successfully");
}

void AddWayPoint::transformPointsViz(std::string frame)
{
  ROS_INFO_STREAM("The frame we want the points in is " << frame << " the frame we are currently in is" << target_frame_);
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform(frame, target_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  target_frame_.assign(frame);

  // TODO Call setCartParams
  std::vector<tf::Transform> waypoints_pos_copy;

  const geometry_msgs::Transform constTransform = transformStamped.transform;
  ROS_DEBUG_STREAM("transform: " << transformStamped);
  tf::Transform transform_old_new;
  tf::transformMsgToTF(constTransform, transform_old_new);

  for (int i = 0; i < waypoints_pos.size(); i++)
  {
    waypoints_pos_copy.push_back(waypoints_pos[i]);
    waypoints_pos_copy[i] = transform_old_new * waypoints_pos_copy[i];
  }
  clearAllPointsRViz();
  for (int i = 0; i < waypoints_pos_copy.size(); i++)
    makeArrow(waypoints_pos_copy[i], i);

  waypoints_pos = waypoints_pos_copy;
  //delete the waypoints_pos vector
  server->applyChanges();
}

void AddWayPoint::clearAllPointsRViz()
{
  waypoints_pos.clear();
  server->clear();
  //delete the waypoints_pos vector
  count = 0;
  makeInteractiveMarker();
  makePointsInteractiveMarker();
  server->applyChanges();
}

void AddWayPoint::modifyMarkerControl(std::string mesh_name, geometry_msgs::Pose object_pose){

  InteractiveMarker interaction_marker;
  InteractiveMarkerControl control_button;

  // Get markers
  if (!server->get("add_point_button", interaction_marker)){
    ROS_ERROR("Could not get marker add_points_button");
    return;
  }
  control_button = interaction_marker.controls.at(0);
  
  // Update control button
  mesh_name_ = mesh_name;
  control_button.markers[0] = makeMeshResourceMarker(mesh_name, object_pose);

  // Update marker pose 
  interaction_marker.pose = object_pose;
  

  // Update parent_home
  parent_home_ = object_pose;

  // Update server
  interaction_marker.controls.at(0) = control_button;
  server->insert(interaction_marker);
  server->setCallback(interaction_marker.name, boost::bind(&AddWayPoint::processFeedbackInter, this, _1));
  server->applyChanges();
}

void AddWayPoint::wayPointOutOfIK_slot(int point_number,int out, std::vector<geometry_msgs::Pose> out_of_bounds_poses)
{
  InteractiveMarker int_marker;
  visualization_msgs::Marker point_marker;
  std::stringstream marker_name;
  marker_name << point_number;
  if (!server->get(marker_name.str(), int_marker)){
    ROS_ERROR_STREAM("Could not get marker with ID: "<<marker_name.str());
    return;
  }

  int control_size = int_marker.controls.size();
  ROS_DEBUG_STREAM("size of controls for marker: " << control_size);

  if (control_size == 0)
  {
    ROS_ERROR("The control size is zero, this is not allowed");
    return;
  }
  else
  {
    control_size = control_size - 1;
  }

  int_marker.controls.at(control_size).markers.erase(int_marker.controls.at(control_size).markers.begin()+1, int_marker.controls.at(control_size).markers.end());
  
  if(out == 1)
  {
    //make the marker outside the IK solution with yellow color
    int_marker.controls.at(control_size).markers.at(0).color = WAY_POINT_COLOR_OUTSIDE_IK;
  }
  else
  {
    int_marker.controls.at(control_size).markers.at(0).color = WAY_POINT_COLOR;
  }
  if (control_size > 2){ 
    // Only do additional step if its selected
    int oob_marker_count = 0;
    for (geometry_msgs::Pose out_of_bounds_pose : out_of_bounds_poses){
      oob_marker_count++;
      Marker oob_marker;
      oob_marker.type = Marker::CUBE;
      oob_marker.header.frame_id = "base_link";
      // oob_marker.ns = "oob_marker";
      // oob_marker.id = oob_marker_count;
      // oob_marker.action = Marker::ADD;
      oob_marker.pose = out_of_bounds_pose;
      oob_marker.scale.x = 0.02;
      oob_marker.scale.y = 0.02;
      oob_marker.scale.z = 0.02;
      oob_marker.color = WAY_POINT_COLOR_OUTSIDE_IK;
      oob_marker.lifetime = ros::Duration(1);
      oob_marker.frame_locked = true;

      // Header header                        # header for time/frame information
      // string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
      // int32 id                           # object ID useful in conjunction with the namespace for manipulating and deleting the object later
      // int32 type                         # Type of object
      // int32 action                         # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
      // geometry_msgs/Pose pose                 # Pose of the object
      // geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
      // std_msgs/ColorRGBA color             # Color [0.0-1.0]
      // duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
      // bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

      int_marker.controls.at(control_size).markers.push_back(oob_marker);
    }
  }
  int_marker.controls.at(control_size);
  server->insert(int_marker);
  server->applyChanges();
}

void AddWayPoint::getRobotModelFrame_slot(const std::string robot_model_frame, const tf::Transform end_effector)
{

  /*! Set the frame of the all the InteractiveMarkers to correspond to the base of the loaded Robot Model.
      This function also initializes the count of the Way-Points and adds the User Interactive Marker to the scene and the RQT Widget.
  */

  target_frame_.assign("base_link");
  ROS_INFO_STREAM("The robot model frame is: " << target_frame_);
  // box_pos = end_effector;

  clearAllPointsRViz();

  count = 0;
  makeInteractiveMarker();
  makePointsInteractiveMarker();
  server->applyChanges();
}

void AddWayPoint::addPoseOffset(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_offset){
  pose_offset.orientation = pose_in.orientation;
  pose_offset.position.x = pose_in.position.x + 0.2;
  pose_offset.position.y = pose_in.position.y + 0.2;
  pose_offset.position.z = pose_in.position.z - 0.2;
}

void AddWayPoint::CheckAllPointsIK(){
  ROS_INFO("Checking IK for all points");
  for(int i = 1; i <= waypoints_pos.size(); i++){
    Q_EMIT onUpdatePosCheckIkValidity(waypoints_pos, i);
  }
}

void AddWayPoint::RobotIKPlanning(const double upper_limit, const double lower_limit, const double step_size, const double h_current){
  ROS_INFO("Checking IK for robot states");

  double h_lower_limit = lower_limit;
  double h_upper_limit = upper_limit;
  double h_step = step_size;
  double h = h_current;

  std::vector<double> delta_hs;
  geometry_msgs::Transform base_link_world_tfmsg;
  tf::Transform T, base_link_world_tf, transformed_waypoint;
  std::vector<tf::Transform> transformed_waypoints;
  tf::Vector3 translation = tf::Vector3(0,0,0);
  Eigen::Affine3d check_ik_point;

  // Get heights
  getDeltaH(h_lower_limit, h_upper_limit, h_step, h, delta_hs);

  // IK checking
  std::vector<bool> ik_result;
  std::vector<double> ik_success_rate;
  // Get baselink transform
  try{
    base_link_world_tfmsg = tfBuffer.lookupTransform("base_link", "map" , ros::Time(0)).transform;
  }
  catch (tf2::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  tf::transformMsgToTF(base_link_world_tfmsg, base_link_world_tf);

  //Initialize vectors
  tf::Transform empty_tf;
  for(int i = 0; i < waypoints_pos.size(); i++){
    transformed_waypoints.push_back(empty_tf);
    ik_result.push_back(false);
  }
  
  // Loop through states
  ROS_INFO("IK Results");
  for(double& delta_h : delta_hs){
    /*
    Apply transformation to all waypoints
    At the end, transformed_waypoint is in base_link frame
    */
    for(int i = 0; i < waypoints_pos.size(); i++){
      //ROS_INFO_STREAM(  waypoints_pos.at(i).getOrigin()[0] << " "<< waypoints_pos.at(i).getOrigin()[1]<<" "<<  waypoints_pos.at(i).getOrigin()[2]);
      addHeight(waypoints_pos[i],delta_h, transformed_waypoint);
      transformed_waypoint = base_link_world_tf * transformed_waypoint;
      transformed_waypoints[i] = transformed_waypoint;
      tf::transformTFToEigen(transformed_waypoint, check_ik_point);
      // Check IK
      ik_result[i] = jaco3_kinematics::ik_exists(check_ik_point, 150);
      addIKValidityMarker(transformed_waypoint, ik_result[i], i);
    }
    printIKInformation(delta_h, h, ik_result);
  } 
  
}

void AddWayPoint::getDeltaH(const double h1, const double h2, const double h_step, const double h, std::vector<double> & delta_hs){
  double n = (h2 - h1)/h_step + 1;
  double abs_h;
  delta_hs.clear();

  for(int i = 0 ; i < n; i++){
    abs_h = h1 + i*h_step;
    delta_hs.push_back(double(abs_h - h));
  }
}

void AddWayPoint::addHeight(const tf::Transform start, const double delta_h, tf::Transform& end){
  end = start;
  tf::Vector3 origin = end.getOrigin();
  origin[2] += delta_h;
  end.setOrigin(origin);
}

void AddWayPoint::printIKInformation(const double delta_h, const double h, const std::vector<bool> ik_result){
  int n = ik_result.size();
  int success_count = 0;
  bool success = true;

  for(auto ik : ik_result){
    if(ik){
      success_count += 1;
    }
    else{
      success = false;
    }
  }
  ROS_INFO_STREAM("Height: "<<(delta_h + h)<<"\tSuccess: "<<success<<"\tRate: "<<100.0*success_count/(n*1.0)<<"%");
}

void AddWayPoint::addIKValidityMarker(const tf::Transform marker_pose, const bool is_valid_ik, const int index){
  visualization_msgs::Marker marker;

  //ROS_INFO_STREAM( marker_pose.getOrigin()[0] << " "<< marker_pose.getOrigin()[1]<<" "<< marker_pose.getOrigin()[2]);
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();

  marker.ns = "ik_checking";
  marker.id = index;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = marker_pose.getOrigin()[0];
  marker.pose.position.y = marker_pose.getOrigin()[1];
  marker.pose.position.z = marker_pose.getOrigin()[2];
  marker.pose.orientation.x = 0;//marker_pose.getRotation().getAxis()[0];
  marker.pose.orientation.y = 0;//marker_pose.getRotation().getAxis()[1];
  marker.pose.orientation.z = 0;//marker_pose.getRotation().getAxis()[2];
  marker.pose.orientation.w = 1;//marker_pose.getRotation().getW();

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  if(is_valid_ik){
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
  }
  else{
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8;
  }

  marker.lifetime = ros::Duration(20);
  marker_pub_.publish(marker);
}

} // namespace moveit_cartesian_plan_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_cartesian_plan_plugin::AddWayPoint, rviz::Panel)
