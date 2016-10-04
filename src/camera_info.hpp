#ifndef CAMERA_INFO_MULTICAMERA__
#define CAMERA_INFO_MULTICAMERA__

#include <iostream>
#include <openni2/OpenNI.h>
#include "openni2_camera/openni2_driver.h"
#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_exception.h"
#include <openni2_camera/openni2_device.h>
#include <vector>
#include <iomanip>
#include <sstream>

struct device_info {
  device_info() {
    downsample = false;
  }
  
  std::string camera_name;
  std::string serial;
  std::string vendor_id;
  std::string product_id;
  std::string location;
  
  bool downsample;
  
  std::string toString() const {
    std::ostringstream os;
    os << "Camera name: " << camera_name << "\n";
    os << "Vendor ID: " << vendor_id << "\n";
    os << "Product ID: " << product_id<< "\n";
    os << "Serial: " << serial << "\n";
    os << "Location: " << location << "\n";
    return os.str(); 
  }
};

std::string getUsbLocation(const std::string& device_uri)
{
  std::string ret = "/dev/bus/usb/";
  int i;
  
  std::string bus_device = device_uri.substr(device_uri.find('@') + 1, device_uri.size());
  std::istringstream bus_is (bus_device.substr(0, bus_device.find('/')));
  bus_is >> i;
  std::ostringstream bus_id;
  bus_id << std::setfill('0') << std::setw(3) << i;
  
  std::istringstream is (bus_device.substr(bus_device.find('/') + 1, bus_device.size()));
  
  is >> i;
  std::ostringstream device_id;
  device_id << std::setfill('0') << std::setw(3) << i;
  
  ret.append(bus_id.str());
  ret.append("/");
  ret.append(device_id.str());
  
  return ret;
}

// Translates the URI of the device into a path: /dev/bus/usb/...
std::vector <device_info> getCamerasInfo() {
  std::vector<device_info> ret_val;
  openni2_wrapper::OpenNI2DeviceManager manager;
  std::string device_id;
   
  // Get the connected OpenNI2 devices
  boost::shared_ptr<std::vector<openni2_wrapper::OpenNI2DeviceInfo> > device_infos = manager.getConnectedDeviceInfos();
  
  // Iterate over the devices, asking the user for labels and generating the proper include tag
  for (size_t i = 0; i < device_infos->size(); ++i)
  {
    try {
      openni2_wrapper::OpenNI2DeviceInfo &info = device_infos->at(i);
      device_info camera_info;
      
      std::ostringstream os, os2;
      os << "camera_" << i;
      std::string camera_label(os.str());
      
      os2 << "#" << i + 1;
      device_id = os2.str();
      
      // Save the camera info
      std::ostringstream product_id, vendor_id;
      product_id << std::hex << std::setfill('0') << std::setw(4) << info.product_id_;
      camera_info.product_id = product_id.str();
      vendor_id << std::hex << std::setfill('0') << std::setw(4) << std::hex << info.vendor_id_;
      camera_info.vendor_id = vendor_id.str();
      camera_info.location = getUsbLocation(info.uri_);
      camera_info.serial = manager.getSerial(info.uri_);
      ret_val.push_back(camera_info);
    } catch(std::exception &e) {
      std::cerr << "Exception thrown while managing camera " << device_id << ". Content: " << e.what() << std::endl;
    }
  }
  
  return ret_val;
}

#endif