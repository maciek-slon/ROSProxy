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
		camera_info_topic("camera_info_topic", std::string("/camera/camera_info")) {
	registerProperty(image_topic);
	registerProperty(camera_info_topic);

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
	ros::spinOnce();
}

void CameraSubscriber::callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& ci) {
	CLOG(LNOTICE) << "Newimage";
	cv::Mat image = cv_bridge::toCvShare(img, "bgr8")->image.clone();
	
	CLOG(LNOTICE) << "Newimage";
	out_img.write(image);
	
	CLOG(LNOTICE) << "Newimage";
	Types::CameraInfo camera_info(ci->width, ci->height, ci->K[0], ci->K[4], ci->K[2], ci->K[5]);
	
	CLOG(LNOTICE) << "Newimage";
	cv::Mat D = cv::Mat::zeros(1, 5, CV_32FC1);
	/*for (int i = 0; i < 5; ++i) D.at<float>(0, i) = ci->D[i];
	camera_info.setDistCoeffs(D);*/
	
	CLOG(LNOTICE) << "Newimage";
	out_camera_info.write(camera_info);
}



} //: namespace CameraSubscriber
} //: namespace Processors
