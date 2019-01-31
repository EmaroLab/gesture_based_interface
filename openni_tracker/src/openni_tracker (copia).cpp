#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include "std_msgs/String.h"

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

ros::Publisher head_kinect_pub;
ros::Publisher left_hand_kinect_pub;
ros::Publisher right_hand_kinect_pub;

ros::Publisher head_baxter_pub;
ros::Publisher left_hand_baxter_pub;
ros::Publisher right_hand_baxter_pub;

ros::ServiceClient client_reset;  


void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);

}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;
    
    // Trasformation from world_frame to camera_link
	tf::TransformListener listener;
	tf::StampedTransform transformation;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s", child_frame_id.c_str());

    tf::Transform transform, transform_kinect, transform_baxter;
    
	// Initialize tranformation from kinect depth camera frame to target
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // Change from kinect depth camera frame to camera link frame
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setRPY(M_PI/2, 0, M_PI/2);
    change_frame.setRotation(frame_rotation);
    
     // Rotate
    tf::Transform change_frame2;
    change_frame2.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation2;
    frame_rotation2.setEulerZYX(-M_PI/2, -M_PI/2, 0);
    change_frame2.setRotation(frame_rotation2);

    transform = transform * change_frame2;
    
    if(child_frame_id.compare("head") != 0)
    {
		 // Rotates
		change_frame2.setOrigin(tf::Vector3(0, 0, 0));
		tf::Quaternion frame_rotation2;
		frame_rotation2.setEulerZYX( M_PI, M_PI, 0);
		change_frame2.setRotation(frame_rotation2);

		transform = transform * change_frame2;
	}
    
    // Update tranformation from camera_link frame to target frame
    transform_kinect = change_frame * transform;
    // Publish in broadcast the transformation
    br.sendTransform(tf::StampedTransform(transform_kinect, ros::Time::now(), frame_id, child_frame_no));
    
    // Inizialize odometry message (wrt Kinect)
    nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = frame_id;
	tf::poseTFToMsg(transform_kinect, odom_msg.pose.pose);
	
	// Find right topic and publish message
    if(child_frame_id.compare("head") == 0){
		head_kinect_pub.publish(odom_msg);
	}
	else if(child_frame_id.compare("left_hand") == 0){
		left_hand_kinect_pub.publish(odom_msg);
	}
	else if(child_frame_id.compare("right_hand") == 0){
		right_hand_kinect_pub.publish(odom_msg);
	}
	
	try{    
		listener.waitForTransform("world_frame", "camera_link", ros::Time(0), ros::Duration(1.0) );
		listener.lookupTransform("world_frame", "camera_link", ros::Time(0), transformation);
		
		// Update tranformation from world frame (Baxter) to target frame
		transform_baxter = transformation * change_frame * transform;
		
		std::string world_frame = "world_frame";
		
		// Publish in broadcast the transformation
		// br.sendTransform(tf::StampedTransform(transform_baxter, ros::Time::now(), world_frame, child_frame_no));
		
		// Inizialize odometry message (wrt Baxter)
		odom_msg.header.stamp = ros::Time::now();
		odom_msg.header.frame_id = "world_frame";
		tf::poseTFToMsg(transform_baxter, odom_msg.pose.pose);
		
		// Find right topic and publish message
		if(child_frame_id.compare("head") == 0){
			head_baxter_pub.publish(odom_msg);
		}
		else if(child_frame_id.compare("left_hand") == 0){
			left_hand_baxter_pub.publish(odom_msg);
		}
		else if(child_frame_id.compare("right_hand") == 0){
			right_hand_baxter_pub.publish(odom_msg);
		}
	}
	catch (tf::TransformException ex){
		ROS_WARN("Transform unavailable %s", ex.what());
	}
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;
        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "right_hand");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "left_hand");
    }
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int main(int argc, char **argv) {
    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh;
    
    // Initilize Publishers for the odometry wrt Kinect
	head_kinect_pub = nh.advertise<nav_msgs::Odometry>("/odometry/kinect/head", 10);
	left_hand_kinect_pub = nh.advertise<nav_msgs::Odometry>("/odometry/kinect/right_hand", 10);
	right_hand_kinect_pub = nh.advertise<nav_msgs::Odometry>("/odometry/kinect/left_hand", 10);
	
    // Initilize Publishers for the odometry wrt Baxter
	head_baxter_pub = nh.advertise<nav_msgs::Odometry>("/odometry/baxter/head", 10);
	left_hand_baxter_pub = nh.advertise<nav_msgs::Odometry>("/odometry/baxter/right_hand", 10);
	right_hand_baxter_pub = nh.advertise<nav_msgs::Odometry>("/odometry/baxter/left_hand", 10);
              
    // Initialize Client to reset kinect
    client_reset = nh.serviceClient<std_srvs::Empty>("reset_kinect_filters");  

	// Set Parameter to tracker
    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
	    if (nRetVal != XN_STATUS_OK) {
		    ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
	    }
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);

	// Acquire parameter
	ros::NodeHandle pnh("~");
	string frame_id("camera_link");
	pnh.getParam("camera_frame_id", frame_id);
	
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		ros::spinOnce();
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
