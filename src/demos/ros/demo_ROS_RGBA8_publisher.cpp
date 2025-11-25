// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Demo to show the use of Chrono::Sensor with ROS (without Chrono::ROS)
//
// =============================================================================


#include "chrono/core/ChTypes.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
using namespace chrono::sensor;

#include <memory>
#include <random>
#include <thread>
#include <vector>
#include <string>
#include <ctime>        // for std::time

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;


// ---- Parameters ---- //
// Switches
bool viz = true;
bool activate_ROS = false;

// Simulation system
double sim_end_time = 1000; // [sec]
double sim_time_step = 2e-4; // [sec]

// ROS parameters
const std::string node_name = "rgba8_publisher";
const std::string topic_name = "rgba8_image";
double ROS_publish_rate = 30; // [Hz], ROS image_message publishing rate

// camera parameters
float cam_update_rate = 30; // [Hz], camera update rate
unsigned int cam_w = 1280; // [px], image width
unsigned int cam_h = 720; // [px], image height
float cam_hFOV = 60 * CH_PI/180.; // [rad], camera horizontal field of view (FOV)
float cam_rot_speed = 0.3; // [rad/sec], camera rotation speed
const std::string cam_name = "camera"; 

// Given time stamp in second, return time stamp in ROS format
builtin_interfaces::msg::Time GetROSTimestamp(double time_s) {
    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = static_cast<int32_t>(time_s);
    timestamp.nanosec = static_cast<uint32_t>((time_s - timestamp.sec) * 1e9);
    return timestamp;
}

class RGBA8PublisherNode : public rclcpp::Node {
	public:
		explicit RGBA8PublisherNode(const std::string node_name = "", std::string topic_name = "rgba8_image")
			: rclcpp::Node(node_name)
		{
			m_publisher = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
		}

		/// Call this from the main loop to publish the image message
		void PublishImgMsg(sensor_msgs::msg::Image &img_msg) {
			m_publisher->publish(img_msg);
		}

	private:
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;
};

int main(int argc, char **argv) {
    
	// Create the system
    ChSystemNSC sys;

	// --------------------- //
	// ---- Scene setup ---- //
	// --------------------- //

	// Add a mesh object to make the scene interesting
	// (filename, load_normals, load_uv)
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/audi/audi_chassis.obj"), false, true);
    mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Audi Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(ChVector3d(0, 0, 0)));
    mesh_body->SetFixed(true);
    sys.Add(mesh_body);

    // This is the body we'll attach the sensors to
    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetFixed(false);
    ground_body->SetMass(0);
    sys.Add(ground_body);

    printf("Scene created.\n");

	// ---------------------- //
	// ---- Sensor setup ---- //
	// ---------------------- //
	
    chrono::ChFrame<double> offset_pose({-8, 0, 2}, QuatFromAngleAxis(.2, {0, 1, 0}));
    
    // Create the sensor system
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(&sys);

	// Add lights
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});

    // Create a camera that's placed on the hood
	// ChCameraSensor(parent_body_attached_to, update_rate, offset_pose, img_w, img_h, horizontal field of view (hFOV))
    auto cam = chrono_types::make_shared<ChCameraSensor>(ground_body, cam_update_rate, offset_pose, cam_w, cam_h, cam_hFOV);
	cam->SetName(cam_name);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
	if (viz) {
    	cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_w, cam_h));
	}
	
    sensor_manager->AddSensor(cam);
	
    // ------------------- //
	// ---- ROS setup ---- //
	// ------------------- //
	// Initialize an RGBA8 image 
	// const int width		= 640;
	// const int height	= 480;
	const int ch		= 4; // RGBA 
	uint8_t rgba8_image[cam_h * cam_w * ch] = {0};

	// Initialize image message format
	sensor_msgs::msg::Image img_msg;
	img_msg.header.frame_id	= cam_name;								// camera's name
	img_msg.width			= cam->GetWidth();
	img_msg.height			= cam->GetHeight();
	img_msg.encoding		= "rgba8";								// RGBA, 8 bits per channel
	img_msg.is_bigendian	= false;
	img_msg.step			= sizeof(PixelRGBA8) * img_msg.width;	// 4 bytes per pixel (R, G, B, A)
	img_msg.data.resize(img_msg.step * img_msg.height);

	// Initialize ROS
	rclcpp::init(argc, argv);

	// Create the ROS node, passing in the initialized image
	auto node = std::make_shared<RGBA8PublisherNode>(node_name, topic_name);

	// Explicit executor
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	
	
	// ------------------------ //
	// ---- Simulation loop --- //
	// ------------------------ //
	std::uniform_int_distribution<> dis(0, 255);

    double sim_time = 0; // [sec]
	ground_body->SetAngVelParent({0, 0, cam_rot_speed}); // [rad/sec]
    
    double publish_frame_time = (ROS_publish_rate == 0) ? 0 : (1 / ROS_publish_rate); // NOTE: If update_rate == 0, tick is called each time
	double time_elapsed_since_last_publish = 0;
	// rclcpp::Rate ROS_rate(10.0); // [Hz]
	
	while (sim_time < sim_end_time) {
        sim_time = sys.GetChTime();
		
		// Fill image with random bytes (0–255)
		// std::mt19937 gen(static_cast<unsigned int>(std::time(nullptr)));
		// for (auto &px : rgba8_image) {
		// 	px = static_cast<uint8_t>(dis(gen));
		// }

        // Update image message data and publish out
		img_msg.header.stamp = GetROSTimestamp(sim_time);
		// img_msg.data.assign(rgba8_image, rgba8_image + img_msg.step * img_msg.height); // debug

		UserRGBA8BufferPtr rgba8_buffer_ptr = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();	// start of PixelRGBA8[]
		if (rgba8_buffer_ptr && rgba8_buffer_ptr->Buffer) {
			const uint8_t* rgba8_ptr = reinterpret_cast<const uint8_t*>(reinterpret_cast<const PixelRGBA8*>(rgba8_buffer_ptr->Buffer.get()));
			img_msg.data.assign(rgba8_ptr, rgba8_ptr + img_msg.step * img_msg.height);
		}
    	
		time_elapsed_since_last_publish += sim_time_step;
		if (time_elapsed_since_last_publish > publish_frame_time) {
			node->PublishImgMsg(img_msg);
			time_elapsed_since_last_publish -= publish_frame_time;
		}
		
		if (!rclcpp::ok())
			break;

		// Update states
		executor.spin_some();
		sensor_manager->Update();
		sys.DoStepDynamics(sim_time_step);
		// ROS_rate.sleep();
    }

	rclcpp::shutdown();
	

	return 0;
}