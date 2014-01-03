#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <cstdlib>
#include <boost/filesystem.hpp>
#include "config.hpp"

#include "record_rgbd_service/SaveImage.h"

using namespace std;
namespace fs = boost::filesystem;
using namespace util;


// parameters
int downsample = 1;
bool verbose = true;
bool display = false;

string camera_name;
string device_id;

bool saving = false;
string folder_name = "";
string camera_folder;
vector<double> stamps;
int save_counter = 0;
int total_counter = 0;


int sizes[2] = {480, 640};
cv::Mat rgb_mat(2, sizes, CV_8UC3);
cv::Mat depth_mat(2, sizes,CV_16UC1);


bool imageSaveCallback(record_rgbd_service::SaveImage::Request &req,
		       record_rgbd_service::SaveImage::Response &res) {

  if (!req.start && saving) {

    saving = false;
    string stampfile_name = folder_name + string("/stamps.txt");
    ofstream stampfile(stampfile_name.c_str());
    stampfile << setprecision(20);
    for (int i=0; i < stamps.size(); ++i) 
      stampfile << stamps[i] << endl;
    stamps.clear();
    stampfile.close();

    cout << "Saved "<<camera_name<<" data to "<<folder_name<<"."<<endl;
   
  } else if (req.start) {
    folder_name = req.folder_name + string("/") + camera_folder;
    saving = true;
    save_counter = 0;
    total_counter = 0;
    stamps.clear(); //Just in case, again.

    cout<<"Saving data to %s." << folder_name <<endl;
  }

  return true;

}



void callback (const boost::shared_ptr<openni_wrapper::Image>& rgb, const boost::shared_ptr<openni_wrapper::DepthImage>& depth, float constant) {

  if (saving) {
    if (total_counter % downsample == 0) {
      stamps.push_back(ros::Time::now().toSec());
      const XnDepthPixel* depth_data = depth->getDepthMetaData().Data(); //unsigned short

      rgb->fillRGB(640, 480, rgb_mat.data, 640*3);
      cv::cvtColor(rgb_mat, rgb_mat, CV_BGR2RGB);

      depth->fillDepthImageRaw(640, 480, (unsigned short*) depth_mat.data);

      bool success1 = cv::imwrite( folder_name + (boost::format("/depth%05d.jpg")%save_counter).str(), depth_mat);
      bool success2 = cv::imwrite( folder_name + (boost::format("/rgb%05d.jpg")%save_counter).str(), rgb_mat);
      if (!success1 || !success2) throw std::runtime_error("Failed to write image.");
      if (verbose) printf("Saved rgb/depth images %i.\n", save_counter);

      if (display) {
	cv::imshow("Hello", rgb_mat);
	cv::waitKey(1);
      }

      save_counter++;
    }

    total_counter++;
  }
}




int main(int argc, char** argv) {
  ros::init(argc, argv, "record_rgbd_video", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_pub;

  nh.getParam("camera_name", camera_name);
  nh.getParam("device_id", device_id);

  int downsample_param;
  if (nh.getParam("downsample", downsample_param)) downsample = downsample_param;


  camera_folder = string("camera_#")+camera_name[6];

  pcl::OpenNIGrabber interface(device_id);
    // connect callback function for desired signal. In this case its a point cloud with color values
  boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float)> f(&callback);
  boost::signals2::connection c = interface.registerCallback (f);

  cv::Size size(640, 480);

  interface.start();

  ros::ServiceServer saveService =
    nh_pub.advertiseService ("saveImages"+camera_name, imageSaveCallback);
    
  cout<<"Spawned service and depth sensor." <<endl;

  ros::spin();
  
  printf("Done.\n");
}

