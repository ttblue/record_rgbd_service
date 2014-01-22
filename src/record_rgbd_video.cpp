#include <fstream>
#include <cstdlib>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>

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
bool publishing = false;
string folder_name = "";
string camera_folder;
vector<double> stamps;
int save_counter = 0;
int total_counter = 0;

ros::Publisher pub;
//pcl::OpenNIGrabber interface;

int sizes[2] = {480, 640};
cv::Mat rgb_mat(2, sizes, CV_8UC3);
cv::Mat depth_mat(2, sizes,CV_16UC1);

typedef union  {
  struct
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  uint32_t long_value;
} RGBValue;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr
convertToPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
		     const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc (new pcl::PointCloud<pcl::PointXYZRGB>());

  pc->width = 640; 
  pc->height = 480; 
  int hw = pc->height * pc->width;
  pc->points.resize (hw); 


  static unsigned rgb_array_size = 0;
  static boost::shared_array<unsigned char> rgb_array ((unsigned char*)(NULL));
  static unsigned char* rgb_buffer = 0;

  // here we need exact the size of the point cloud for a one-one correspondence!
  if (rgb_array_size < hw * 3)
  {
    rgb_array_size = hw * 3;
    rgb_array.reset (new unsigned char [rgb_array_size]);
    rgb_buffer = rgb_array.get ();
  }
  image->fillRGB (pc->width, pc->height, rgb_buffer, pc->width * 3);
  
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  register const unsigned short* depth_map = depth_image->getDepthMetaData ().Data (); 
  if (depth_image->getWidth() != pc->width || depth_image->getHeight () != pc->height) { 
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer (0); 
    
    if (buffer_size < hw) { 
      buffer_size = hw; 
      depth_buffer.reset (new unsigned short [buffer_size]); 
    } 
    depth_image->fillDepthImageRaw (pc->width, pc->height, depth_buffer.get ()); 
    depth_map = depth_buffer.get (); 
  }

  // fill in the RGB values
  int value_idx = 0;
  RGBValue color;
  unsigned char Alpha = 0;

  register float constant = 1.0f / 525; 
  register int centerX = (pc->width >> 1); 
  int centerY = (pc->height >> 1); 
  register int depth_idx = 0; 
  for (int v = -centerY; v < centerY; ++v) 
    { 
      for (register int u = -centerX; u < centerX; ++u, ++depth_idx, value_idx += 3) 
        { 
	  pcl::PointXYZRGB& pt = pc->points[depth_idx]; 

	  color.Red   = rgb_buffer[value_idx];
	  color.Green = rgb_buffer[value_idx + 1];
	  color.Blue  = rgb_buffer[value_idx + 2];
      
	  pt.rgba = color.long_value;

	  //This part is used for invalid measurements, I removed it 
	  if (depth_map[depth_idx] == 0 || 
	      depth_map[depth_idx] == depth_image->getNoSampleValue () || 
	      depth_map[depth_idx] == depth_image->getShadowValue ()) 
	    { 
	      // not valid 
	      pt.x = pt.y = pt.z = bad_point; 
	      continue; 
	    }
	  pt.z = depth_map[depth_idx] * 0.001f; 
	  pt.x = static_cast<float> (u) * pt.z * constant; 
	  pt.y = static_cast<float> (v) * pt.z * constant; 

        } 
    } 
  pc->sensor_origin_.setZero (); 
  pc->sensor_orientation_.w () = 0.0f; 
  pc->sensor_orientation_.x () = 1.0f; 
  pc->sensor_orientation_.y () = 0.0f; 
  pc->sensor_orientation_.z () = 0.0f; 

  return pc;
}



bool imageSaveCallback(record_rgbd_service::SaveImage::Request &req,
		       record_rgbd_service::SaveImage::Response &res) {

  if (req.publish) {
    cout<<"Publishing pointclouds." <<endl;
    publishing = true;
  }
  else publishing = false;

  if (!req.start and saving) {
    saving = false;
    string stampfile_name = folder_name + string("/stamps.txt");
    ofstream stampfile(stampfile_name.c_str());
    stampfile << setprecision(20);
    for (int i=0; i < stamps.size(); ++i) 
      stampfile << stamps[i] << endl;
    stamps.clear();
    stampfile.close();

    cout << "Saved "<<camera_name<<" data to "<<folder_name<<"."<<endl;
  } else if (req.start && !saving) {
    folder_name = req.folder_name + string("/") + camera_folder;
    saving = true;
    save_counter = 0;
    total_counter = 0;
    stamps.clear(); //Just in case, again.

    cout<<"Saving data to " << folder_name <<"."<<endl;
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

      bool success1 = cv::imwrite( folder_name + (boost::format("/depth%05d.png")%save_counter).str(), depth_mat);
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

  if (publishing) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc =  convertToPointCloud  (rgb, depth);
    pcl::PCLPointCloud2 pcl_pc;
    sensor_msgs::PointCloud2 sm_pc;

    pcl::toPCLPointCloud2(*pc, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, sm_pc);
    sm_pc.header.frame_id=camera_name+"_rgb_optical_frame";
    pub.publish(sm_pc);
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

  std::string topic_name = std::string("PointCloud_")+camera_name;
  pub = nh_pub.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
    
  cout<<"Spawned service and depth sensor." <<endl;

  ros::spin();
  
  printf("Done.\n");
}

