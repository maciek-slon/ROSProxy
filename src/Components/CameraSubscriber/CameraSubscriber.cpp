/*!
 * \file
 * \brief
 * \author Maciej Stefanczyk
 */

#include <memory>
#include <string>

#include "CameraSubscriber.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CameraSubscriber {

CameraSubscriber::CameraSubscriber(const std::string & name) :
		Base::Component(name) , 
		image_topic("image_topic", std::string("/camera/image_raw")), 
		camera_info_topic("camera_info_topic", std::string("/camera/camera_info")),
		image_type("image_type", std::string("bgr8")),
		spin("ros.spin", true),
		K(3, 3, CV_64FC1),
		R(3, 3, CV_64FC1),
		D(1, 5, CV_64FC1),
		T(3, 1, CV_64FC1),
		P(3, 4, CV_64FC1) {
	registerProperty(image_topic);
	registerProperty(camera_info_topic);
	registerProperty(image_type);
	registerProperty(spin);

	new_image = false;
}

CameraSubscriber::~CameraSubscriber() {
}

void CameraSubscriber::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_img", &out_img);
	registerStream("out_camera_info", &out_camera_info);
	// Register handlers
	registerHandler("spinOnce", boost::bind(&CameraSubscriber::spinOnce, this));
	addDependency("spinOnce", NULL);

}

bool CameraSubscriber::onInit() {
  static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, "image_listener", ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	it = new image_transport::ImageTransport(*nh);
	
	sub = it->subscribeCamera(image_topic, 1, &CameraSubscriber::callback, this);

	return true;
}

bool CameraSubscriber::onFinish() {
	return true;
}

bool CameraSubscriber::onStop() {
	return true;
}

bool CameraSubscriber::onStart() {
	return true;
}

void CameraSubscriber::spinOnce() {
	if (spin)
		ros::spinOnce();
	
	if (new_image) {
		CLOG(LINFO) << "Sending...";
		out_img.write(img.clone());
		out_camera_info.write(ci);
		
		new_image = false;
	}
}

void CameraSubscriber::callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& ci) {
	CLOG(LINFO) << name();
	
	cv::Mat image = cv_bridge::toCvShare(img, image_type)->image.clone();
	
	Types::CameraInfo camera_info(ci->width, ci->height, ci->K[0], ci->K[4], ci->K[2], ci->K[5]);
	
	for (int i = 0; i < 9; ++i) K.at<double>(i/3, i%3) = ci->K[i]; 
	camera_info.setCameraMatrix(K);
	CLOG(LDEBUG) << K;
	
	
	
	
	
	
	for (int i = 0; i < 9; ++i) R.at<double>(i/3, i%3) = ci->R[i]; 
	camera_info.setRotationMatrix(R);
	CLOG(LDEBUG) << R;
	
	for (int i = 0; i < 5; ++i) D.at<double>(0, i) = ci->D[i]; 
	camera_info.setDistCoeffs(D);
	CLOG(LDEBUG) << D;
	
	
	
	
	
	
	T.at<double>(0, 0) = ci->P[3] / ci->P[0];
	T.at<double>(1, 0) = 0;
	T.at<double>(2, 0) = 0;
	camera_info.setTranlationMatrix(T);
	CLOG(LDEBUG) << T;
	
	for (int i = 0; i < 12; ++i) P.at<double>(i/4, i%4) = ci->P[i]; 
	camera_info.setProjectionMatrix(P);
	CLOG(LDEBUG) << P;
	
	this->img = image.clone();
	this->ci = camera_info;
	new_image = true;
}



} //: namespace CameraSubscriber
} //: namespace Processors
