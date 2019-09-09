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
#include <tf2_ros/transform_listener.h>
#include <peanut_cotyledon/GetCleanPath.h>
#include <peanut_cotyledon/CleanPath.h>
#include <peanut_cotyledon/GetCleanPathRequest.h>
#include <peanut_elevator_oil/MoveToHeight.h>

// macros
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

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
			ros::ServiceClient get_clean_path_proxy_;
  			ros::Publisher robot_goal_pub;

			// Elevator and navigation services
			ros::ServiceClient move_elevator_;
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
			//! Initialize the TreeView with the User Interactive Marker.
		    void initTreeView();
		    //! Handle the event of a Way-Point deleted from the RQT UI.
			void pointDeletedUI();
			//! Handle the event of a Way-Point added from the RQT UI.
			void pointAddUI();
			//! Insert a row in the TreeView.
			void insertRow(const tf::Transform& point_pos,const int count);
			//! Remove a row in the TreeView.
			void removeRow(int marker_nr);
			//! Handle the event when a User updates the pose of a Way-Point through the RQT UI.
			void pointPosUpdated_slot( const tf::Transform& point_pos, const char* marker_name);
			//! Get the selected Way-Point from the RQT UI.
			void selectedPoint(const QModelIndex& current, const QModelIndex& previous);
			//! Handle the even when the data in the TreeView has been changed.
			void treeViewDataChanged(const QModelIndex &index,const QModelIndex &index2);
			//! Slot for when the play until button is pressed
			void playUntilPointBtn();
			//! Slot for parsing the Way-Points and notifying the MoveIt.
			void parseWayPointBtn_slot();
			
			void visualizeGoalConfig();

			//! Send a signal that a save the Way-Points to a file button has been pressed.
			void savePointsToFile();
			//! Send a signal that a load the Way-Points from a file button has been pressed.
			void loadPointsFromFile();
			//! slot connected to clear all the boxes for interaction around points
			void clearAllInteractiveBoxes_slot();
			//! Slot connected to a clear all points button click.
			void clearAllPoints_slot();
			void transformPointsToFrame();
			//! Set the start pose of the User Interactive Marker to correspond to the loaded robot base frame.
			void setAddPointUIStartPos(const std::string robot_model_frame,const tf::Transform end_effector);
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

			//! Set the planning group ComboBox
			void getCartPlanGroup(std::vector< std::string > group_names);

			void selectedPlanGroup(int index);

			//! Create a slot to call a signal on which the Move the robot to home position function is called
			void moveToHomeFromUI();
			void parsePlanConfigBtn_slot();
			void parsePlanExecuteConfigBtn_slot();

			// Slots for elevator 
			void moveElevator();
			void moveElevatorHelper();
		Q_SIGNALS:
			//! Notify RViz enviroment that a new Way-Point has been added from RQT.
		    void addPoint( const tf::Transform point_pos );

		    //! Notify RViz enviroment that a new Way-Point has been deleted from RQT.
		    void pointDelUI_signal( std::string marker_name);
		    //! Notify RViz enviroment that a new Way-Point has been modified from RQT.
		    void pointPosUpdated_signal( const tf::Transform& position, const char* marker_name);
			void parseWayPointBtnGoto_signal(int start_index, int stop_index);
		    //! Signal to notify the Cartesian Path Planning Class that an Execute Cartesian Plan button has been pressed.
		    void parseWayPointBtn_signal();

			void configEdited_signal(std::vector<double> config);
			void parseConfigBtn_signal(std::vector<double> config, bool plan_only);
		    //! Save to file button has been pressed.
		    void saveToFileBtn_press(std::string floor_name, std::string area_name, int object_id, std::string task_name, peanut_cotyledon::CleanPath clean_path);
		    //! Signal that clear all points button has been pressed.
		    void clearAllPoints_signal();
			void transformPointsViz(std::string frame);
			//! signal that the clear all boxes button has been pressed.
			void clearAllInteractiveBoxes_signal();
		    //! Signal that the Cartesian Plan execution button has been pressed.
		    void cartesianPathParamsFromUI_signal(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_, std::string robot_model_frame_, bool fix_start_state);

		    //! On this signal we will call the function for which will exectute the MoveIt command to bring the robot in its initial state.
		    void moveToHomeFromUI_signal();

			void sendSendSelectedPlanGroup(int index);

		};
	}


} //end of namespace moveit_cartesian_plan_plugin

#endif //path_planning_widget_H_
