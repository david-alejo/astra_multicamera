// This utility will act as an aid in order to identify the position of the cameras and automatically generate 
// a xml which will be used by the multicamera driver in order to emit the data of each camera with its correspoding label

#include <string>
#include <iostream>
#include <tinyxml.h>
#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_exception.h"
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif


#include <boost/shared_ptr.hpp>
#include <boost/concept_check.hpp>

using namespace std;
using openni2_wrapper::OpenNI2DeviceManager;
using openni2_wrapper::OpenNI2DeviceInfo;
using openni2_wrapper::OpenNI2Exception;

bool saveXMLFile(const string& s);
TiXmlElement *getXmlCameraElement(openni2_wrapper::OpenNI2DeviceInfo info, const std::string &serial);

openni2_wrapper::OpenNI2DeviceManager manager;

using namespace std;

int main (int argc, char **argv) {
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <config_filename>\n";
    return -1;
  }
  
  if (saveXMLFile(string(argv[1]))) {
    cout << "Config file exported successfully.\n";
  }
  
  return 0;
}


bool saveXMLFile(const string &s) {
  bool ret_val = true;
   
  TiXmlDocument doc;
  TiXmlDeclaration decl( "1.0", "", "" );  
  doc.InsertEndChild( decl );  
  
  // Create root node
  boost::shared_ptr<std::vector<openni2_wrapper::OpenNI2DeviceInfo> > device_infos = manager.getConnectedDeviceInfos();
  
  std::cout << "Found " << device_infos->size() << " devices:" << std::endl << std::endl;
  for (size_t i = 0; i < device_infos->size(); ++i)
  {
    cout << "Generating xml for device # " << i << std::endl;
    try {
      std::string serial = manager.getSerial(device_infos->at(i).uri_);
      doc.LinkEndChild(getXmlCameraElement(device_infos->at(i), serial));
    }
    catch (const OpenNI2Exception& exception)
    {
      std::cerr << "Could not retrieve serial number: " << exception.what() << std::endl;
    }

  }
  
  try {
    doc.SaveFile(s);
  } catch (exception &e) {
    std::cerr << "Exception catched while writing the XML document to the file: " << s << std::endl;
    ret_val = false;
  }
  
  return ret_val;
}

TiXmlElement *getXmlCameraElement(openni2_wrapper::OpenNI2DeviceInfo info, const std::string &serial) {
  TiXmlElement *ret = new TiXmlElement("camera");
  ret->SetAttribute("product_id", std::to_string(info.product_id_));
  ret->SetAttribute("vendor_id", std::to_string(info.vendor_id_));
  ret->SetAttribute("uri", info.uri_);
  ret->SetAttribute("serial", serial);
  
  return ret;
}
