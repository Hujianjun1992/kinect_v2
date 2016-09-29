#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

/// [headers]

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr cloud ( new PointCloud );
PointCloud::Ptr cloud_nan ( new PointCloud );
PointCloud::Ptr cloud_down ( new PointCloud );


int vp_1,vp_2;

pcl::visualization::PCLVisualizer *pcl_viewer = (new pcl::visualization::PCLVisualizer("PCL_VIEWER"));

pcl::VoxelGrid<PointT> voxel;


void break_point(std::string string)
{
  std::cout << " \t break_point " << string << "\t" << std::endl;
}

bool protonect_shutdown = false; ///< Whether the running application should shut down.

/// [logger]
#include <fstream>
#include <cstdlib>
class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
  {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
  {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
  {
    //    break_point("hujianjun");
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};
/// [logger]
void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
{
  if(event.keyDown())
    {
      //      std::cout << "\t" << "event.getKeySym()" << "----------->" << event.getKeySym() << std::endl;
    }
  if(event.getKeySym() == "space" && event.keyDown())
    {
      std::cout << "\t" << "xuhong" << std::endl;
      //next_iteration = true;
    }
}

int main(int argc, char *argv[])
/// [main]
{
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  MyFileLogger *filelogger = new MyFileLogger("logfile");//(getenv("LOGFILE"));
  if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);
  else
    delete filelogger;


  /// [context]
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  /// [context]

  std::string serial = "";

  bool enable_rgb = true;
  bool enable_depth = true;
  size_t framemax = -1;

  /// [discovery]
  if(freenect2.enumerateDevices() == 0)
    {
      std::cout << "no device connected!" << std::endl;
      return -1;
    }

  if (serial == "")
    {
      serial = freenect2.getDefaultDeviceSerialNumber();
    }
  /// [discovery]


  dev = freenect2.openDevice(serial);

  if(dev == 0)
    {
      std::cout << "failure opening device!" << std::endl;
      return -1;
    }

  protonect_shutdown = false;

  /// [listeners]
  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  /// [listeners]

  /// [start]
  if (enable_rgb && enable_depth)
    {
      if (!dev->start())
        return -1;
    }
  else
    {
      if (!dev->startStreams(enable_rgb, enable_depth))
        return -1;
    }

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
  std::cout << "device IrCameraParams: " << dev->getIrCameraParams().k1 << std::endl;
  //  std::cout << "device ColorCameraParams: " << dev->getColorCameraParams() << std::endl;
  /// [start]

  /// [registration setup]
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
  /// [registration setup]

  ///PCL VIEWER///
  pcl_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp_1);
  pcl_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp_2);
  pcl_viewer->addPointCloud(cloud, "vp_1_cloud");
  pcl_viewer->addPointCloud(cloud_down, "vp_2_cloud");
  pcl_viewer->setBackgroundColor(0.0, 0.05, 0.05, vp_1);
  pcl_viewer->setBackgroundColor(0.05, 0.05, 0.05, vp_2);
  pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp_1_cloud");
  pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vp_2_cloud");

  pcl_viewer->addCoordinateSystem(.1);
  pcl_viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  //pcl_viewer->initCameraParameters();
  pcl_viewer->registerKeyboardCallback(&keyboardEvent,(void*)NULL);
  size_t framecount = 0;

  /// [loop start]
  while(!protonect_shutdown && (framemax == (size_t)-1 ||framecount < framemax))
    {
      if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
        {
          std::cout << "timeout!" << std::endl;
          return -1;
        }
      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
      /// [loop start]

      if (enable_rgb && enable_depth)
        {
          registration->apply(rgb, depth, &undistorted, &registered);
          PointT point;
          float RGB;

          for(int m = 0; m < depth->height; m++)
            for(int n = 0; n < depth->width; n++)
              {
                registration->getPointXYZRGB(&undistorted, &registered, m, n, point.x, point.y, point.z, RGB);

                const uint8_t *p = reinterpret_cast<uint8_t*>(&RGB);
                point.b = p[0];
                point.g = p[1];
                point.r = p[2];
                cloud->points.push_back(point);
              }
          cloud->height = 1;
          cloud->width = cloud->points.size();
          cloud->is_dense = false;
          {
            pcl_viewer->updatePointCloud(cloud, "vp_1_cloud");
            std::cout << "\t" << "cloud_sise_before" << "--------->" << cloud->width*cloud->height << "\t" << std::endl;
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud_nan, indices);
            std::cout << "\t" << "cloud_sise_before" << "--------->" << cloud_nan->width*cloud_nan->height << "\t" << std::endl;
            std::cout << "\t" << "indices" << "--------->" << indices.size() << "\t" << std::endl;
            voxel.setInputCloud(cloud_nan);
            voxel.setLeafSize(.5f, .5f, .5f);
            voxel.filter(*cloud_down);
            std::cout << "\t" << "cloud_sise_after" << "--------->" << cloud_down->width*cloud_down->height << "\t" << std::endl;

            pcl_viewer->updatePointCloud(cloud_down, "vp_2_cloud");
            //            std::cout << "\t" << "pcl_viewer->wasStopped()" << "\t" << pcl_viewer->wasStopped() << std::endl;
            pcl_viewer->spinOnce();
          }
          cloud->points.clear();
          cloud_nan->points.clear();
          cloud_down->points.clear();
          /// [registration]
        }

      framecount++;



      protonect_shutdown = protonect_shutdown || pcl_viewer->wasStopped();

      listener.release(frames);
    }

  /// [stop]
  dev->stop();
  dev->close();
  /// [stop]

  delete registration;

  return 0;
}
