// This utility will act as an aid in order to identify the position of the cameras and generates two files: and automatically generate 
// 1- A ROS launch file which can be used to start all the ROS video streams
// 2- 

#include <string>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <tinyxml.h>
#include <openni2/OpenNI.h>
#include "openni2_camera/openni2_driver.h"
#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_exception.h"
#include <openni2_camera/openni2_device.h>
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// OpenCV to ROS stuff
#include <cv_bridge/cv_bridge.h>

#include <boost/shared_ptr.hpp>
#include <boost/concept_check.hpp>

using namespace std;
using openni2_wrapper::OpenNI2DeviceManager;
using openni2_wrapper::OpenNI2DeviceInfo;
using openni2_wrapper::OpenNI2Exception;

openni2_wrapper::OpenNI2DeviceManager manager;

void newColorFrameCallback(sensor_msgs::ImagePtr image); // Get the images from the OpenNI2 stream

// Global flag stuff... the demon is inside hahahaha
int img_num = 0;
bool wait = false;
int max_wait = 10; // Maximum wait time in seconds
string device_id; // ID of the current camera

struct device_info {
  string camera_name;
  string serial;
  string vendor_id;
  string product_id;
  string location;
  
  string toString() const {
    ostringstream os;
    os << "Camera name: " << camera_name << "\n";
    os << "Vendor ID: " << vendor_id << "\n";
    os << "Product ID: " << product_id<< "\n";
    os << "Serial: " << serial << "\n";
    os << "Location: " << location << "\n";
    return os.str();
  }
};

vector <device_info> getCamerasInfo();
bool saveLaunchFile(const string& s, const vector <device_info> &cameras, const string &input_file);
void getDefaultParametersFromLaunchFile(const std::string &launch_file, TiXmlElement *launch_element);
bool saveUdevRules(const string &s, const vector <device_info> &cameras);
// Translates the URI of the device into a path: /dev/bus/usb/...
string getUsbLocation(const string &device_uri);
string getSerialNumber(const string &device_location);
void addArgumentTags(TiXmlElement& elem_add, const TiXmlElement& elem_source);

int main (int argc, char **argv) {
  if (argc != 3) {
    cerr << "Usage: " << argv[0] << " <launch_filename> <input_filename>\n";
    return -1;
  }
  
  const std::string window_name = "image_show";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE ); // Create a window for display.
  
  vector<device_info> camera_names = getCamerasInfo(); // Save the camera names
  
  if (camera_names.size() > 0) {
    cout << "Detected " << camera_names.size() << " cameras. Info:\n";
    
    for (uint i = 0; i < camera_names.size(); i++) {
      cout << camera_names.at(i).toString();
    }
    
    cout << "Saving launch file.\n";
    
    if (saveLaunchFile(string(argv[1]), camera_names, string(argv[2]))) {
      cout << "Launch file saved successfully.\n";
    }
  }
  
  cv::destroyWindow("image_show");
  
  return 0;
}

// Uses tinyxml to generate a launch file with 
bool saveLaunchFile(const string &s, const vector<device_info> &camera_info_vec, const string &input_file) {
  bool ret_val = true;
  // Create document
  TiXmlDocument doc;
  TiXmlDeclaration decl( "1.0", "", "" );  
  doc.InsertEndChild( decl );  
  
  // Create root launch node
  TiXmlElement launch_element("launch");
  
  getDefaultParametersFromLaunchFile(input_file, &launch_element);

  for (int i = 0; i < camera_info_vec.size(); i++) {
    const device_info &curr_cam = camera_info_vec.at(i);
    TiXmlElement *include_elem = new TiXmlElement("include");
    
    // File attribute: the default launch file of openni2
    include_elem->SetAttribute("file", "$(find openni2_launch)/launch/openni2.launch");
    
    // Tag argument 1: device_id
    TiXmlElement *arg_tag = new TiXmlElement("arg");
    arg_tag->SetAttribute("name", "device_id");
    arg_tag->SetAttribute("value", curr_cam.serial);
    include_elem->LinkEndChild(arg_tag);
    
    // Second tag --> argument name of the camera
    arg_tag = new TiXmlElement("arg");
    arg_tag->SetAttribute("name", "camera");
    arg_tag->SetAttribute("value", curr_cam.camera_name);
    include_elem->LinkEndChild(arg_tag);
    
    addArgumentTags(*include_elem, launch_element);
    
    launch_element.LinkEndChild(include_elem);
  }
  
  doc.InsertEndChild(launch_element);
  doc.SaveFile(s);
  
  return ret_val;
}

vector<device_info> getCamerasInfo()
{
  vector<device_info> ret_val;
   
  // Get the connected OpenNI2 devices
  boost::shared_ptr<std::vector<openni2_wrapper::OpenNI2DeviceInfo> > device_infos = manager.getConnectedDeviceInfos();
  std::cout << "Found " << device_infos->size() << " devices:" << std::endl;
  
  // Iterate over the devices, asking the user for labels and generating the proper include tag
  for (size_t i = 0; i < device_infos->size(); ++i)
  {
    openni2_wrapper::OpenNI2DeviceInfo &info = device_infos->at(i);
    device_info camera_info;
    
    ostringstream os, os2;
    os << "camera_" << i;
    std::string camera_label(os.str());
    
    os2 << "#" << i + 1;
    device_id = os2.str();
    
    cout << "Showing the RGB image associated with camera " << device_id << endl;

    try {
      img_num = 0;
      wait = true;
      boost::shared_ptr<openni2_wrapper::OpenNI2Device> device = manager.getDevice(info.uri_);
      
      if (device->hasColorSensor()) {
	device->setColorFrameCallback(newColorFrameCallback);
	device->startColorStream();
      }
      
      int cont = 0;
      // Wait for the image to be shown and a key pressed
      while (wait && cont < max_wait) {
	cont++;
	sleep(1);
      }
      if (wait) {
	// No image has been found
	cerr << "Warning: Could not retrieve image from camera " << device_id << ". Setting the label to camera_" << i + 1 << endl;
	wait = false;
      } else {
	cout << "Please enter the label for camera " << device_id << endl;
	getline(cin, camera_label);
      }
      
      // Save the camera info
      camera_info.camera_name = camera_label;
      std::ostringstream product_id, vendor_id;
      product_id << std::hex << setfill('0') << setw(4) << info.product_id_;
      camera_info.product_id = product_id.str();
      vendor_id << std::hex << setfill('0') << setw(4) << std::hex << info.vendor_id_;
      camera_info.vendor_id = vendor_id.str();
      camera_info.location = getUsbLocation(info.uri_);
      camera_info.serial = manager.getSerial(info.uri_);
      ret_val.push_back(camera_info);
    } catch (exception &e) {
      cerr << "Exception thrown while managing camera " << device_id << ". Content: " << e.what() << endl;
    }
  }
  
  return ret_val;
}

// Get the accepted arguments and their default values from another launch file (typically openni2_launch/launch/openni2.launch)
void getDefaultParametersFromLaunchFile(const std::string &launch_file, TiXmlElement *launch_element) {
  // Load the file where the default parameters will be stored
  TiXmlDocument doc(launch_file);
  doc.LoadFile();
  TiXmlElement *root = doc.RootElement();
  
  if (!root) {
    cerr << "Could not get the launch file.\n";
    return;
  }
  
  // Iterate over the children and copy the argument data
  TiXmlNode *it = root->FirstChild();
  while (it) {
    if (it->ValueStr() == "arg" && it->ToElement()) {
      string name(it->ToElement()->Attribute("name"));
      // Discard undesired tags
      if (name != "camera" && name != "rgb_frame_id" && name != "device_id" && name != "rgb_frame_id" && name != "depth_frame_id" && name != "depth_camera_info_url" &&
	name != "rgb_camera_info_url")
      {
	TiXmlElement *node = new TiXmlElement("arg");
      
	node->SetAttribute("name", it->ToElement()->Attribute("name"));
	node->SetAttribute("default", it->ToElement()->Attribute("default"));
	
	if (it->ToElement()->Attribute("if")) {
	  node->SetAttribute("if", it->ToElement()->Attribute("if"));
	} else if (it->ToElement()->Attribute("unless")) {
	  node->SetAttribute("unless", it->ToElement()->Attribute("unless"));
	}
	
	launch_element->LinkEndChild(node);
      }
    }
    // Next getXmlCameraElement
    it = root->IterateChildren(it);
  }
}

void newColorFrameCallback(sensor_msgs::ImagePtr image)
{
  if (img_num == 0 && wait) {
    img_num++;
    cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(image, "bgr8");

    ostringstream os;
    os << "Camera " << device_id;
    
    cv::imshow("image_show", img_cv.get()->image );                   // Show our image inside it.
    cv::updateWindow ("image_show");
    cv::startWindowThread();
    cv::waitKey(0); // Wait for a key to be pressed in the window
    wait = false;
  }
}

string getUsbLocation(const string& device_uri)
{
  string ret = "/dev/bus/usb/";
  int i;
  
  string bus_device = device_uri.substr(device_uri.find('@') + 1, device_uri.size());
  istringstream bus_is (bus_device.substr(0, bus_device.find('/')));
  bus_is >> i;
  ostringstream bus_id;
  bus_id << setfill('0') << setw(3) << i;
  
  istringstream is (bus_device.substr(bus_device.find('/') + 1, bus_device.size()));
  
  is >> i;
  ostringstream device_id;
  device_id << setfill('0') << setw(3) << i;
  
  ret.append(bus_id.str());
  ret.append("/");
  ret.append(device_id.str());
  
  
  return ret;
}

void addArgumentTags(TiXmlElement& elem_add, const TiXmlElement& elem_source)
{
  // Iterate over the children and copy the argument data
  const TiXmlNode *it = elem_source.FirstChild();
  while (it) {
    if (it->ValueStr() == "arg" && it->ToElement() && static_cast<string>(it->ToElement()->Attribute("name")) != "device_id") {
      TiXmlElement *node = new TiXmlElement("arg");
      
      node->SetAttribute("name", it->ToElement()->Attribute("name"));
      ostringstream os;
      os << "$(arg " << it->ToElement()->Attribute("name") << ")";
      node->SetAttribute("value", os.str());
      elem_add.LinkEndChild(node);
    }
    // Next getXmlCameraElement
    it = elem_source.IterateChildren(it);
  }
}

