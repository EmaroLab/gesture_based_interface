#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <math.h>

//transformation from Baxter to Kinect

/** 
 * Callback of the /move_kinect service
 * @param[in]  req  Request Message
	* @param[in]  req.angle Desired Orientation angle of the Kinect
 * @param[out]  res    Response of the service
	* @param[out]  res.result If the operation is completed successfully
 */
double x_kinect = 0.0;
double y_kinect = 0.0;
double z_kinect = 0.0;

double angle = 20.0;

class TF_Broadcaster{
    /** TF_Broadcaster:
     * - subscribe to /cur_tilt_angle to know the current tilt angle of the Kinect
     */
    public:
    TF_Broadcaster(){
        angle_sub = nh.subscribe("cur_tilt_angle", 2, &TF_Broadcaster::angleCB, this);
    }
    /**
     * Angle callback function
     * acquires the current tilt angle of the Kinect that is useful to know the right background to remove
     * @param[in]  angle_msg	current tilt angle of the Kinect
     */
    void angleCB(const std_msgs::Float64& angle_msg){
        if(angle_msg.data <= 30.0 && angle_msg.data >= -30.0)
        {
                angle = round(angle_msg.data);
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber angle_sub;  /**< Subscriber to /cur_tilt_angle */
};
    
/**
 * Main
 */
main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle n;
	
	n.param<double>("x_kinect", x_kinect, 0.3);
	n.param<double>("y_kinect", y_kinect, 0.3);
	n.param<double>("z_kinect", z_kinect, 0.3);
	
	TF_Broadcaster tf_broadcaster;
	
	ros::Rate r(20000);
	tf::TransformBroadcaster broadcaster;
	
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(x_baxter, y_baxter, z_baxter));

	tf::Quaternion frame_rotation;
        
    float rad_angle;
	while(ros::ok()){
		rad_angle = angle * M_PI / -180;
		frame_rotation.setRPY(0, rad_angle, 0); //pitch = current tilt angle of the Kinect
		change_frame.setRotation(frame_rotation);
		
		broadcaster.sendTransform(
			tf::StampedTransform(change_frame, ros::Time::now(), "world_frame", "camera_link"));
				
		ros::spinOnce();
		r.sleep();
	}

    return 0;
}
