


#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseArray.h>

#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

#include <QObject>
#include <QTimer>
#include <QtConcurrentRun>
#include <QFuture>




#ifndef PLACE_BASE_H_
#define PLACE_BASE_H_



/*!
 *  \brief     Class for setting up the Base Placement Environment.
 *  \details   The PlaceBase Class handles all the interactions between the widget and the actual task.
 	 		   This Class inherits from the QObject superclass.
 	 		   The concept of this class is to initialize all the necessary parameters for generating Optimal Base Locations from task poses.
 	 		   
 *  \author    Abhijit Makhal
 */

class PlaceBase: public QObject
{
Q_OBJECT

public:

public:
    //! Constructor for the Base Placement Planner.
    PlaceBase(QObject* parent = 0);
    //! Virtual Destructor for the Base Placement planner.
    virtual ~PlaceBase();
    //! Initialization of the necessary parameters.
    void init();
	
public Q_SLOTS:
    //!Getting the reachability data from widget
    void setReachabilityData(std::multimap<std::vector<double>, std::vector<double> > PoseCollection, std::multimap<std::vector<double>, double > SphereCollection, float resolution);

    //!Showing the InverseReachability Map
    void ShowUnionMap(bool show_map);

    //!Clearing the InverseReachability Map
    void clearUnionMap(bool clear_map);

    //!Showing the base locations by arrow
    void showBaseLocationsbyArrow(std::vector<geometry_msgs::Pose> po);

    //! Showing the base locations by robot model
    void showBaseLocationsbyRobotModel(std::vector<geometry_msgs::Pose> po);
   
    //! Get the Way-Points from the RViz enviroment and use them to find base.
    void findbase(std::vector<geometry_msgs::Pose> grasp_poses);
  
    //! Sending the initial marker location and list of base Placement Method and visualization methods
    void initRvizDone();

    //! Function for setting base placement mainfunction to a separate thread.
    void BasePlacementHandler(std::vector<geometry_msgs::Pose> waypoints);

    //! Setting the Base Placement Paremeters
    void setBasePlaceParams(int base_loc_size, int high_score_sp_);

    //! Getting user Defined base placement method
    void getSelectedMethod(int index);

    //! Getting user Defined visualization method
    void getSelectedOpType(int op_index);

Q_SIGNALS:
	
    //! Signal for initial marker frame
    void getinitialmarkerFrame_signal(const tf::Transform trns);

    //! Let the RQT Widget know that a Base Placement Process has started.
    void basePlacementProcessStarted();;
	
    //! Let the RQT Widget know that Base Placement Process has finished.
    void basePlacementProcessFinished();
	
    //! Let the RQT Widget know that Base Placement Process completed so it can show finish message
    void basePlacementProcessCompleted();
        
    //! Send the Method groups to the GUI
    void sendBasePlaceMethods_signal(std::vector< std::string > method_names);

    //! Send the output types to the GUI
    void sendOuputType_signal(std::vector< std::string > output_type);

       

protected:
          
    //! Base Placement by PCA 
    void findBaseByPCA();

    //! Base Placement by GraspReachabilityScore
    void findBaseByGraspReachabilityScore();

    //!Base Placement by IKSolutionScore
    void findBaseByIKSolutionScore();

    //! Selecting the Base Placement Method
    void BasePlaceMethodHandler();

    //! Selecting the Visualization Method
    void OuputputVizHandler(std::vector<geometry_msgs::Pose> po);

    
    int selected_method_;
    int selected_op_type_;
	    
    //! Number of base place locations to show
    int BASE_LOC_SIZE_;
    //! Number of High Scoring Spheres
    int HIGH_SCORE_SP_;

    //! Vector for method_names
    std::vector<std::string> method_names_;
    //! Vector for output visualization
    std::vector<std::string> output_type_;
    //! Taking the reachability data from the file  
    std::multimap<std::vector<double>, std::vector<double> > PoseColFilter;
    std::multimap<std::vector<double>, double > SphereCol;
    float res;

    //! Reachability data for the union map
    std::multimap<std::vector<double>, std::vector<double> >baseTrnsCol;
    std::map<std::vector<double>, double>  sphereColor; 

    //! sphere centers based on their scores
    std::vector<std::vector<double> > highScoreSp;
    //! Vector for storing final base poses
    std::vector<geometry_msgs::Pose> final_base_poses;

   //! Vector for grasp poses
   std::vector<geometry_msgs::Pose> GRASP_POSES_;
};

#endif // PLACE_BASE_H_
