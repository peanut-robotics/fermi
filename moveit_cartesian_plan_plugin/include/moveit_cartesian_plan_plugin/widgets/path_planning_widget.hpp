#ifndef path_planning_widget_H_
#define path_planning_widget_H_

#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string.h>
#include <actionlib/client/simple_action_client.h>

#include <ui_path_planning_widget.h>

#include <moveit_cartesian_plan_plugin/add_way_point.hpp>

#include <QWidget>
#include <QTimer>
// #include <QtConcurrentRun>
#include <QtConcurrent/QtConcurrent>
#include <QMainWindow>
#include <QTreeView>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QSplitter>
#include <QHeaderView>
#include <QCompleter>
#include <QIntValidator>
#include <QDataStream>
#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressBar>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ObjectColor.h>
#include <std_msgs/ColorRGBA.h>
#include <moveit_msgs/ObjectColor.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <map>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <QInputDialog>
#include <exception>

// Cotyledon
#include <peanut_cotyledon/GetFloors.h>
#include <peanut_cotyledon/SetFloor.h>
#include <peanut_cotyledon/GetAreas.h>
#include <peanut_cotyledon/SetArea.h>
#include <peanut_cotyledon/GetCleanPath.h>
#include <peanut_cotyledon/SetCleanPath.h>
#include <peanut_cotyledon/CleanPath.h>
#include <peanut_cotyledon/Object.h>
#include <peanut_cotyledon/GetObjects.h>
#include <peanut_cotyledon/GetObject.h>
#include <peanut_cotyledon/SetObject.h>
#include <peanut_cotyledon/GetCleanPathRequest.h>
#include <peanut_cotyledon/Object.h>
#include <peanut_cotyledon/Task.h>
#include <peanut_cotyledon/GetTasks.h>
#include <peanut_cotyledon/GetTask.h>
#include <peanut_cotyledon/SetTask.h>
#include <peanut_cotyledon/CachedPath.h>
#include <peanut_cotyledon/Floor.h>

#include <peanut_elevator_oil/MoveToHeightAction.h>
#include <peanut_navplanning_oil/MoveBaseAction.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <controller_manager_msgs/SwitchController.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <eigen_conversions/eigen_msg.h>

// macros
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

Q_DECLARE_METATYPE(trajectory_msgs::JointTrajectory)

namespace moveit_cartesian_plan_plugin
{
	namespace widgets {

/*!
 *  \brief     Class for handling the User Interactions with the RQT Widget.
 *  \details   The PathPlanningWidget Class handles all User Interactions with the RQT GUI.
 	 		   This Class inherits from the QWidget superclass.
 	 		   The concept of the RQT widget is to add the same possabilities as the UI from the RViz enviroment and enabling simultanious communication betweet the RViz Enviroment and the RQT GUI.
 *  \author    Risto Kojcev
 */

		class PathPlanningWidget: public QWidget
		{
		Q_OBJECT
		public:
			//! RQT Widget Constructor.
			PathPlanningWidget(std::string ns="");
			//! Virtual RQT Widget Destructor.
			virtual ~PathPlanningWidget();
			//! set the name of the RQT Widget.
		    std::string get_name()
			{
				return "RobotPathPlanner";
			}
			ros::NodeHandle nh_;
			ros::ServiceClient get_floors_proxy_;
			ros::ServiceClient set_floor_proxy_;
			ros::ServiceClient get_areas_proxy_;
			ros::ServiceClient set_area_proxy_;
			ros::ServiceClient get_clean_path_proxy_;
			ros::ServiceClient set_clean_path_proxy_;
			ros::ServiceClient get_objects_proxy_;
			ros::ServiceClient get_object_proxy_;
			ros::ServiceClient set_object_proxy_;
			ros::ServiceClient get_tasks_proxy_;
			ros::ServiceClient get_task_proxy_;
			ros::ServiceClient set_task_proxy_;
			ros::ServiceClient add_task_proxy_;
  			ros::Publisher robot_goal_pub;
			
			// Elevator and navigation services
			ros::ServiceClient clear_faults_;
			ros::ServiceClient switch_controllers_;
			boost::shared_ptr<actionlib::SimpleActionClient<peanut_elevator_oil::MoveToHeightAction>> move_elevator_;
			boost::shared_ptr<actionlib::SimpleActionClient<peanut_navplanning_oil::MoveBaseAction>> move_base_;
			
			// Transformations
			tf2_ros::Buffer tfBuffer_;
			tf2_ros::StaticTransformBroadcaster static_broadcaster_;
			boost::shared_ptr<tf2_ros::TransformListener> tfListener_;

		protected:
			//! Widget Initialization.
			void init();
			std::string param_ns_;
			//! Protected variable for the Qt UI to access the Qt UI components.
			Ui::PathPlanningWidget ui_;
			//! Definition of an abstract data model.
			QStandardItemModel* pointDataModel;
		private:
			QStringList pointList;
			//! Checks the range of the points.
			void pointRange();
		protected Q_SLOTS:
			//! Slot for when the play until button is pressed
			void playUntilPointBtn();
			void goToPrev();
			void goToNext();
			//! Slot for parsing the Way-Points and notifying the MoveIt.
			void parseWayPointBtn_slot();
			void parseWayPointBtn_planonly_slot();
			
			void visualizeGoalConfig();

			//! Send a signal that a save the Way-Points to a file button has been pressed.
			void savePointsTool();
			void savePointsObject();
			void loadPointsTool();
			void savePoints();
			void loadPoints();
			void saveRefNavPose();
			//! Send a signal that a load the Way-Points from a file button has been pressed.
			void loadPointsObject();
			//! slot connected to clear all the boxes for interaction around points
			void clearAllInteractiveBoxes_slot();
			//! Slot connected to a clear all points button click.
			void clearAllPoints_slot();
			void clearAllPoints_button_slot();
			void transformPointsToFrame();
			//! Slot for disabling the TabWidged while Cartesian Path is executed.
			void cartesianPathStartedHandler();
			//! Slot for enabling the TabWidged after Cartesian Path is executed.
			void cartesianPathFinishedHandler();
			//! Send the Cartesian and MoveIt parameters to the Cartesian Path Planning class.
			void sendCartTrajectoryParamsFromUI();
			//! Set a label in the RQT to inform the user of the percantage of completion of the Cartesian plan.
			void cartPathCompleted_slot(double fraction);
			//update the point in the RQT by using separate thread
			// void pointPosUpdatedHandler_slot(const tf::Transform& point_pos, const char* marker_name);
			
			void selectedPlanGroup(int index);

			//! Create a slot to call a signal on which the Move the robot to home position function is called
			void moveToHomeFromUI();
			void parsePlanConfigBtn_slot();
			void parsePlanExecuteConfigBtn_slot();

			// Slots for elevator and navigation
			void moveElevator();
			void moveElevatorHelper();
			void addNavPose();
			void addNavPoseHelper();
			void goToNavPose();
			void goToNavPoseHelper();
			void addTaskHelper();

			// Slots for faults
			void clearFaults();
			void stopAll();
			void startController();
			void stopController();
			
			// Slots for check ik
			void ChangeCheckIK();
			void CheckAllPointsIK();

			// Helpers
			bool getObjectWithName(std::string floor_name, std::string area_name, std::string object_name, peanut_cotyledon::Object& desired_obj);
			bool setObjectHelper(std::string floor_name, std::string area_name, std::string object_name, peanut_cotyledon::Object obj);

			void ChangeStepSize();
			void RobotIKPlanning();
			void SetTool();
			void SetMesh();

			// Print trigger information
			void showDeviceTriggerPoints();
			void showDeviceTriggerPoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
			void UpdateTriggerPoint();
			void DeleteTriggerPoint();
			int GetInsertIdx(peanut_cotyledon::CleanPath& clean_path, const int& idx);

			// Get clean path
			bool GetCleanPath(peanut_cotyledon::CleanPath& clean_path);
			bool SetCleanPath(const peanut_cotyledon::CleanPath& clean_path);

			// Point selectin
			void SelectPoint();

			// Drop down menu
			void updateAreaMenu(const QString& string);
			void updateObjectIDMenu(const QString& string);
			void updateTaskMenu(const QString& string);
			void ResetMenu();

			// Add data buttons
			void addFloorCb();
			void addAreaCb();
			void addObjectCb();
			void addTaskCb();
		
			// Save cartesian cached trajectory
			void saveCachedCartesianTrajectory(const trajectory_msgs::JointTrajectory& traj);
			void executeCachedCartesianTrajectory();

		Q_SIGNALS:
			//! Notify RViz enviroment that a new Way-Point has been added from RQT.
		    void addPoint( const tf::Transform point_pos );

		    //! Notify RViz enviroment that a new Way-Point has been deleted from RQT.
		    void pointDelUI_signal( std::string marker_name);
		    //! Notify RViz enviroment that a new Way-Point has been modified from RQT.
		    void pointPosUpdated_signal( const tf::Transform& position, const char* marker_name);
			void parseWayPointBtnGoto_signal(int start_index, int stop_index);
		    //! Signal to notify the Cartesian Path Planning Class that an Execute Cartesian Plan button has been pressed.
		    void parseWayPointBtn_signal(bool plan_only);

			void configEdited_signal(std::vector<double> config);
			void parseConfigBtn_signal(std::vector<double> config, bool plan_only);
		    void saveToolBtn_press();
			//! Save to file button has been pressed.
		    void saveObjectBtn_press(std::string floor_name, std::string area_name, std::string object_name, std::string task_name, peanut_cotyledon::CleanPath clean_path, std::string mesh_name);
		    //! Signal that clear all points button has been pressed.
		    void clearAllPoints_signal();
			// Signal to modify control marker
			void modifyMarkerControl_signal(std::string mesh_name, geometry_msgs::Pose object_pose);
			void transformPointsViz(std::string frame);
			//! signal that the clear all boxes button has been pressed.
			void clearAllInteractiveBoxes_signal();
		    //! Signal that the Cartesian Plan execution button has been pressed.
		    void cartesianPathParamsFromUI_signal(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_, std::string robot_model_frame_, bool fix_start_state);

		    //! On this signal we will call the function for which will exectute the MoveIt command to bring the robot in its initial state.
		    void moveToHomeFromUI_signal();

			void sendSendSelectedPlanGroup(int index);

			void SelectPoint_signal(int idx);

			void ChangeCheckIK_signal();
			void CheckAllPointsIK_signal();
			void RobotIKPlanning_signal(const double upper_limit, const double lower_limit, const double step_size, const double h,
										const double radius, const double radius_step, const double max_angle, const double min_angle, const double angle_step);

			// Signal to modify points control marker
			void ModifyPointsMarkerPose_signal();

			void executeCartesianTrajectory(const trajectory_msgs::JointTrajectory& traj);
		};
	}


} //end of namespace moveit_cartesian_plan_plugin

#endif //path_planning_widget_H_
