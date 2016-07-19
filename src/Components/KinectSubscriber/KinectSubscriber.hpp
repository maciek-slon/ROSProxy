/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef KINECTSUBSCRIBER_HPP_
#define KINECTSUBSCRIBER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/CameraInfo.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace Processors {
namespace KinectSubscriber {

/*!
 * \class KinectSubscriber
 * \brief KinectSubscriber processor class.
 *
 * 
 */
class KinectSubscriber: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	KinectSubscriber(const std::string & name = "KinectSubscriber");

	/*!
	 * Destructor
	 */
	virtual ~KinectSubscriber();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;
	Base::DataStreamOut<cv::Mat> out_depth;
	Base::DataStreamOut<Types::CameraInfo> out_camera_info;

	// Handlers

	// Properties
	Base::Property<std::string> image_topic;
	Base::Property<std::string> depth_topic;
	Base::Property<std::string> camera_info_topic;
	Base::Property<std::string> image_type;
	Base::Property<std::string> depth_type;
	Base::Property<bool> spin;

	
	// Handlers
	void spinOnce();
	
	
	ros::NodeHandle * nh;
	image_transport::ImageTransport * it;
	image_transport::SubscriberFilter *subC, *subD;

	void callback(const sensor_msgs::ImageConstPtr& imgC, const sensor_msgs::ImageConstPtr& imgD);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	
	message_filters::Synchronizer<MySyncPolicy> * sync;

	cv::Mat K,R,D,T,P;
	
	cv::Mat img;
	cv::Mat depth;
	Types::CameraInfo ci;
	
	bool new_image;
};

} //: namespace KinectSubscriber
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("KinectSubscriber", Processors::KinectSubscriber::KinectSubscriber)

#endif /* KINECTSUBSCRIBER_HPP_ */
