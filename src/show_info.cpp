#include "camera_info.hpp"

using namespace std;

int main(int argc, char **argv)
{
  vector<device_info> camera_info_vec = getCamerasInfo(); // Get the camera names and attributes
  
  if (camera_info_vec.size() > 0) {
    cout << "Detected " << camera_info_vec.size() << " cameras. Info:\n";
    
    for (uint i = 0; i < camera_info_vec.size(); i++) {
      cout << camera_info_vec.at(i).toString();
    }
  }
  
  
  return 0;

}

