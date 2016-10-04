// This utility will act as an aid in order to identify the position of the cameras and generates several files 
// 1- A ROS launch file for each camera with the desired options
// 2- A delayed ROS launch that sequentially calls the above generated launch files

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
#include "camera_info.hpp"

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

// Global stuff... the demon is inside hahahaha
int img_num = 0;
bool wait = false;
int max_wait = 10; // Maximum wait time in seconds to get the keyboard response from the user
string device_id; // ID of the current camera
int data_skip = 0; // Data skip values
int delay = 2; // Delay between launching the drivers of two consecutive cameras

vector <device_info> getCamerasInfoWithLabel();
bool saveDelayedLaunchFiles(const string &output_file, const vector<device_info> &camera_info_vec);
bool saveCameraLaunchFile(const device_info &camera_info, const string &input_file);
void getDefaultParametersFromLaunchFile(const std::string &launch_file, TiXmlElement *launch_element);
bool saveUdevRules(const string &s, const vector <device_info> &cameras);
void addArgumentTags(TiXmlElement& elem_add, const TiXmlElement& elem_source);
TiXmlElement *getDriverParameter(const std::string &name, const std::string &value, const std::string &camera_name);

int main (int argc, char **argv) {
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <launch_filename> <input_filename> [<data_skip> [<delay_between_cameras>]]\n";
    cerr << "Data skip means the number of frames necessary in order to publish one. I.e. data skip = 3 will publish 1 image of each 3 received\n";
    cerr << "Delay between cameras: each camera will be activated x seconds after the preceeding one (defaults to 2)\n";
    return -1;
  }
  
  if (argc >3) {
    data_skip = atoi(argv[3]);
  }
  
  const std::string window_name = "image_show";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE ); // Create a window for display.
  
  vector<device_info> camera_info_vec = getCamerasInfo(); // Get the camera names and attributes
  
  if (camera_info_vec.size() > 0) {
    cout << "Detected " << camera_info_vec.size() << " cameras. Info:\n";
    
    for (uint i = 0; i < camera_info_vec.size(); i++) {
      cout << camera_info_vec.at(i).toString();
    }
    
    cout << "Saving launch file.\n";
    
    // Generate the separate camera files
    for (unsigned int i = 0; i < camera_info_vec.size(); i++) {
      if (saveCameraLaunchFile(camera_info_vec.at(i), string(argv[2]))) {
        cout << camera_info_vec.at(i).camera_name << ".launch file saved successfully.\n";
      }
    }
    // Generate the main launch that will call the others
    saveDelayedLaunchFiles(string(argv[1]), camera_info_vec);
    cout << argv[1] << " file saved successfully.\n";
  }
  
  cv::destroyWindow("image_show");
  
  return 0;
}

bool saveDelayedLaunchFiles(const string &output_file, const vector<device_info> &camera_info_vec) {
  bool ret_val = true;
  // Create document
  TiXmlDocument doc;
  TiXmlDeclaration decl( "1.0", "", "" );  
  doc.InsertEndChild( decl );  
  
  // Create root launch node
  TiXmlElement launch_element("launch");
  
  int curr_delay = delay;
  for (unsigned int i = 0; i < camera_info_vec.size(); i++, curr_delay += delay) {
    TiXmlElement *node_elem = new TiXmlElement("node");
    
    // Attribute 1: pkg --> openni2_multicamera
    node_elem->SetAttribute("pkg", "openni2_multicamera");
    // Attribute 2: type --> timed_roslaunch.sh
    node_elem->SetAttribute("type", "timed_roslaunch.sh");
    // Attribute 3: args --> arguments of the script (1 --> delay, 2 --> name of the script)
    ostringstream args_;
    args_ << curr_delay << " openni2_multicamera ";
    args_ << camera_info_vec.at(i).camera_name << ".launch";
    node_elem->SetAttribute("args", args_.str());
    // Attribute 4: name --> camera_name + delay
    string node_name(camera_info_vec.at(i).camera_name);
    node_name.append("_delay");
    node_elem->SetAttribute("name", node_name);
    
    
    launch_element.LinkEndChild(node_elem);
  }
  doc.InsertEndChild(launch_element);
  doc.SaveFile(output_file);
  
  return ret_val;
}

// Uses tinyxml to generate a launch file with 
bool saveCameraLaunchFile(const device_info &camera_info, const string &input_file) {
  bool ret_val = true;
  // Create document
  TiXmlDocument doc;
  TiXmlDeclaration decl( "1.0", "", "" );  
  doc.InsertEndChild( decl );  
  
  // Create root launch node
  TiXmlElement launch_element("launch");
  
  getDefaultParametersFromLaunchFile(input_file, &launch_element);
  
  TiXmlElement *arg_skip_tag = new TiXmlElement("arg");
  arg_skip_tag->SetAttribute("name", "data_skip");
  arg_skip_tag->SetAttribute("default", data_skip);
  launch_element.LinkEndChild(arg_skip_tag);
  TiXmlElement *include_elem = new TiXmlElement("include");
  
  // File attribute: the default launch file of openni2
  include_elem->SetAttribute("file", "$(find openni2_launch)/launch/openni2.launch");
  
  // Tag argument 1: device_id
  TiXmlElement *arg_tag = new TiXmlElement("arg");
  arg_tag->SetAttribute("name", "device_id");
  arg_tag->SetAttribute("value", camera_info.serial);
  include_elem->LinkEndChild(arg_tag);
  
  // Second tag --> argument name of the camera
  arg_tag = new TiXmlElement("arg");
  arg_tag->SetAttribute("name", "camera");
  arg_tag->SetAttribute("value", camera_info.camera_name);
  include_elem->LinkEndChild(arg_tag);
  
  TiXmlElement *param_tag = getDriverParameter("data_skip", "$(arg data_skip)", camera_info.camera_name);
  launch_element.LinkEndChild(param_tag);
  
  if (camera_info.downsample) {
    param_tag = getDriverParameter("color_mode", "8", camera_info.camera_name);
    launch_element.LinkEndChild(param_tag);
    param_tag = getDriverParameter("depth_mode", "8", camera_info.camera_name); // Downsampling to QVGA (320x200)
    launch_element.LinkEndChild(param_tag);
  }
  
  addArgumentTags(*include_elem, launch_element);
  
  launch_element.LinkEndChild(include_elem);

  doc.InsertEndChild(launch_element);
  doc.SaveFile(camera_info.camera_name + ".launch");
  
  return ret_val;
}

TiXmlElement *getDriverParameter(const std::string &name, const std::string &value, const std::string &camera_name) {
  TiXmlElement *param_tag = new TiXmlElement("param");
  string s("/");
  s.append(camera_name);
  s.append("/driver/");
  s.append(name);
  param_tag->SetAttribute("name", s);
  param_tag->SetAttribute("value", value);
  return param_tag;
}

vector<device_info> getCamerasInfoWithLabel()
{
  vector<device_info> ret_val = getCamerasInfo();
   
  // Get the connected OpenNI2 devices
  boost::shared_ptr<std::vector<openni2_wrapper::OpenNI2DeviceInfo> > device_infos = manager.getConnectedDeviceInfos();
  
  // Iterate over the devices, asking the user for labels and generating the proper include tag
  for (size_t i = 0; i < device_infos->size(); ++i)
  {
    openni2_wrapper::OpenNI2DeviceInfo &info = device_infos->at(i);
    device_info camera_info = ret_val.at(i);
    
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
        std::string downsample;
        cout << "Downsample it? (y/N)" << device_id << endl;
        getline(cin, downsample);
        if (downsample[0] == 'y') {
          cout << "Downsampling\n";
          camera_info.downsample = true;
        } else {
          camera_info.downsample = false;
        }
      }
      
      // Update the label of the camera info
      camera_info.camera_name = camera_label;
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

void addArgumentTags(TiXmlElement& elem_add, const TiXmlElement& elem_source)
{
  // Iterate over the children and copy the argument data
  const TiXmlNode *it = elem_source.FirstChild();
  while (it) {
    if (it->ValueStr() == "arg" && it->ToElement() && static_cast<string>(it->ToElement()->Attribute("name")) != "device_id"
      && static_cast<string>(it->ToElement()->Attribute("name")) != "data_skip"
    ) {
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

