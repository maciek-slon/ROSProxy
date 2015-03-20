/*!
 * \file
 * \brief 
 * \author Maciej Stefanczyk
 */

#ifndef CAMERASUBSCRIBER_HPP_
#define CAMERASUBSCRIBER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/CameraInfo.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace Processors {
namespace CameraSubscriber {

/*!
 * \class CameraSubscriber
 * \brief CameraSubscriber processor class.
 *
 * 
 */
class CameraSubscriber: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CameraSubscriber(const std::string & name = "CameraSubscriber");

	/*!
	 * Destructor
	 */
	virtual ~CameraSubscriber();

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
	Base::DataStreamOut<Types::CameraInfo> out_camera_info;

	// Handlers

	// Properties
	Base::Property<std::string> image_topic;
	Base::Property<std::string> camera_info_topic;

	
	// Handlers
	void spinOnce();
	
	
	ros::NodeHandle * nh;
	image_transport::ImageTransport * it;
	image_transport::CameraSubscriber sub;

	void callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& ci);
};

} //: namespace CameraSubscriber
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CameraSubscriber", Processors::CameraSubscriber::CameraSubscriber)

#endif /* CAMERASUBSCRIBER_HPP_ */
