/**
   Code based of gscam's webcam viewer.
   Modified to save images.

   For more information/code on ros-gst interface, please look at gscam's ros wiki.
   Credit goes to them.
 **/

#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <fstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "record_rgbd_service/SaveImage.h"

using namespace std;

//forward declarations
static gboolean processData(GstPad *pad, GstBuffer *buffer, gpointer u_data);

//globals
int width, height;

bool display = false;

bool saving = false;
string folder_name = "";
string camera_name;
string camera_folder;
vector<double> stamps;
int save_counter = 0;

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
    stamps.clear(); //Just in case, again.

    cout<<"Saving data to %s." << folder_name <<endl;
  }

  return true;

}


int main(int argc, char** argv) {

  ros::init(argc, argv, "camera_image_saver");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_pub;


  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  string gsconfig_rosparam = "";
  bool gsconfig_rosparam_defined = false;
  char *gsconfig_env = NULL;
  string config;

  gsconfig_rosparam_defined = nh.getParam("gscam_config",gsconfig_rosparam);
  ROS_INFO(gsconfig_rosparam.c_str());
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    ROS_FATAL( "Problem getting GSCAM_CONFIG environment variable and 'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline." );
    return false;
  } else if(gsconfig_env && gsconfig_rosparam_defined) {
    ROS_FATAL( "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. Please only define one." );
    return false;
  } else if(gsconfig_env) {
    config = std::string(gsconfig_env);
    ROS_INFO_STREAM("Using gstreamer config from env: \""<<gsconfig_env<<"\"");
  } else if(gsconfig_rosparam_defined) {
    config = gsconfig_rosparam;
    ROS_INFO_STREAM("Using gstreamer config from rosparam: \""<<gsconfig_rosparam<<"\"");
  }
  
  gst_init(0,0);
  cout << "Gstreamer Version: " << gst_version_string() << endl;

  GError *error = 0; //assignment to zero is a gst requirement
  GstElement *pipeline = gst_parse_launch(config.c_str(),&error);
  if (pipeline == NULL) {
    cout << error->message << endl;
    exit(-1);
  }

  bool sync_sink;
  bool preroll;

  nh.param("sync_sink", sync_sink, true);
  nh.param("preroll", preroll, false);
  nh.getParam("camera_name", camera_name);

  camera_folder = string("camera_#")+camera_name[6];

  GstElement * sink = gst_element_factory_make("appsink",NULL);
  GstCaps * caps = gst_caps_new_simple("video/x-raw-rgb", NULL);
  gst_app_sink_set_caps(GST_APP_SINK(sink), caps);
  gst_caps_unref(caps);

  gst_base_sink_set_sync(GST_BASE_SINK(sink), (sync_sink) ? TRUE : FALSE);

  if(GST_IS_PIPELINE(pipeline)) {
    GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline), GST_PAD_SRC);
    g_assert(outpad);
    GstElement *outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);


    if(!gst_bin_add(GST_BIN(pipeline), sink)) {
      fprintf(stderr, "gst_bin_add() failed\n"); // TODO: do some unref

      gst_element_set_state(outelement, GST_STATE_NULL);
      gst_object_unref(outelement);

      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      return -1;
    }

    if(!gst_element_link(outelement, sink)) {
      fprintf(stderr, "GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));

      gst_element_set_state(outelement, GST_STATE_NULL);
      gst_object_unref(outelement);

      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      return -1;
    }

    gst_element_set_state(outelement, GST_STATE_NULL);
    gst_object_unref(outelement);

  } else {
    GstElement* launchpipe = pipeline;
    pipeline = gst_pipeline_new(NULL);
    g_assert(pipeline);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline), launchpipe, sink, NULL);

    if(!gst_element_link(launchpipe, sink)) {
      ROS_FATAL("GStreamer: cannot link launchpipe -> sink");

      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      return -1;
   }
  }

  gst_element_set_state(pipeline, GST_STATE_PAUSED);
  
  if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
    return -1;
  } else {
    ROS_DEBUG_STREAM("Stream is PAUSED.");
  }


  if (preroll) {
    ROS_DEBUG("Performing preroll...");
    //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    //I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Failed to PLAY during preroll.");
      return -1 ;
    } else {
      ROS_DEBUG("Stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipeline, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Failed to PAUSE.");
        return -1;
      } else {
        ROS_INFO("Stream is PAUSED in preroll.");
      }  
  }

  ros::ServiceServer saveService =
    nh_pub.advertiseService ("saveImages"+camera_name, imageSaveCallback);
 
  cout << "Ready..." << endl;

  //processVideo
  gst_element_set_state(pipeline, GST_STATE_PLAYING);


  while(nh.ok()) {

    // This should block until a new frame is awake, this way, we'll run at the 
    // actual capture framerate of the device.
    GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink));
    if (!buf) break;

    GstPad* pad = gst_element_get_static_pad(sink, "sink");
    const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
    GstStructure *structure = gst_caps_get_structure(caps,0);
    gst_structure_get_int(structure,"width",&width);
    gst_structure_get_int(structure,"height",&height);

    if (saving) {
      cv::Mat cv_img (height, width, CV_8UC3, const_cast<uchar*>(buf->data), width*3);
      cv::cvtColor(cv_img, cv_img, CV_BGR2RGB);
    
      stamps.push_back(ros::Time::now().toSec());
      bool success = cv::imwrite( (folder_name + (boost::format("/rgb%05d.jpg")%save_counter).str()), cv_img);

      if (!success) throw std::runtime_error("Failed to write image.");
      else printf("saved rgb image %i.\n", save_counter);

      save_counter ++;
      if (display) {
	cv::imshow("Hello", cv_img);
	cv::waitKey(1);
      }
    }

    gst_buffer_unref(buf);
    ros::spinOnce();

  }

  //close out
  cout << "\nquitting..." << endl;
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);

  return 0;
}
