#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>

using boost::asio::ip::tcp;
  

struct PointCloudBuffers
{
  typedef boost::shared_ptr<PointCloudBuffers> Ptr;
  std::vector<short> points;
  std::vector<unsigned char> rgb;
};

void
CopyPointCloudToBuffers (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, PointCloudBuffers& cloud_buffers)
{
  const size_t nr_points = cloud->points.size ();

  cloud_buffers.points.resize (nr_points*3);
  cloud_buffers.rgb.resize (nr_points*3);

  const pcl::PointXYZ  bounds_min (-0.9f, -0.8f, 1.0f);
  const pcl::PointXYZ  bounds_max (0.9f, 3.0f, 3.3f);

  size_t j = 0;
  for (size_t i = 0; i < nr_points; ++i)
  {

    const pcl::PointXYZRGBA& point = cloud->points[i];

    if (!pcl_isfinite (point.x) || 
        !pcl_isfinite (point.y) || 
        !pcl_isfinite (point.z))
      continue;

    if (point.x < bounds_min.x ||
        point.y < bounds_min.y ||
        point.z < bounds_min.z ||
        point.x > bounds_max.x ||
        point.y > bounds_max.y ||
        point.z > bounds_max.z)
      continue;

    const int conversion_factor = 500;

    cloud_buffers.points[j*3 + 0] = static_cast<short> (point.x * conversion_factor);
    cloud_buffers.points[j*3 + 1] = static_cast<short> (point.y * conversion_factor);
    cloud_buffers.points[j*3 + 2] = static_cast<short> (point.z * conversion_factor);

    cloud_buffers.rgb[j*3 + 0] = point.r;
    cloud_buffers.rgb[j*3 + 1] = point.g;
    cloud_buffers.rgb[j*3 + 2] = point.b;

    j++;
  }

  cloud_buffers.points.resize (j * 3);
  cloud_buffers.rgb.resize (j * 3);
}


template <typename PointType>
class PCLMobileServer
{
  public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    
    int port_;
    std::string device_id_;
    boost::mutex mutex_;

    pcl::VoxelGrid<PointType> voxel_grid_filter_;
    pcl::visualization::CloudViewer viewer_;

    CloudPtr filtered_cloud_;
    PointCloudBuffers::Ptr buffers_;

    PCLMobileServer (const std::string& device_id = "", int port = 11111,
                     float leaf_size_x = 0.01, float leaf_size_y = 0.01, float leaf_size_z = 0.01)
    : port_ (port),
      device_id_ (device_id),
      viewer_ ("PCL Viewer")
    {
      voxel_grid_filter_.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
    
    }
    
    void setMat(cv::Mat& depthImage, int depth_min, int depth_max) {

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

        cloud->width = depthImage.cols; //Dimensions must be initialized to use 2-D indexing
        cloud->height = depthImage.rows; 
        cloud->resize(cloud->width*cloud->height); 

        float *pixel = (float*)(depthImage.data);
        int *depth_data = new int[cloud->height * cloud->width];
        //copy the depth values of every pixel in here
        int i=0;
        float depth;
        for(int v=0; v < depthImage.rows; v++)
            for(int u = 0; u < depthImage.cols; u++, i++) {
                depth_data[i] = depthImage.at<float>(v,u); 
                //depth_data[i] = pixel[depthImage.cols*v+u];
                }


        register float constant = 1.0f / 525;
        register int centerX = (cloud->width >> 1);
        int centerY = (cloud->height >> 1);
        register int depth_idx = 0;
        for (int v = -centerY; v < centerY; ++v)
        {
            for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
            {
                depth = depth_data[depth_idx];
                if (depth_min < depth < depth_max) {
                    pcl::PointXYZRGBA& pt = cloud->points[depth_idx];
                    pt.z = depth / 4500.0f; // 0.001f;
                    pt.x = static_cast<float> (u) * pt.z * constant;
                    pt.y = static_cast<float> (v) * pt.z * constant;
                    pt.r = 1.0;
                }
            }
        }
        cloud->sensor_origin_.setZero ();
        cloud->sensor_orientation_.w () = 0.0f;
        cloud->sensor_orientation_.x () = 1.0f;
        cloud->sensor_orientation_.y () = 0.0f;
        cloud->sensor_orientation_.z () = 0.0f; 

        
        //void createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage, cv::Mat& rgbImage, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outputPointcloud, Eigen::Matrix3f& rgbIntrinsicMatrix)
        //float rgbFocalInvertedX = 1/rgbIntrinsicMatrix(0,0);    // 1/fx
        //float rgbFocalInvertedY = 1/rgbIntrinsicMatrix(1,1);    // 1/fy

        /*
        pcl::PointXYZRGBA newPoint;
        for (int i=0; i<depthImage.rows; i++)
        {
            for (int j=0; j<depthImage.cols; j++)
            {
                float depthValue = depthImage.at<float>(i,j);

                if (depthValue == depthValue)                // if depthValue is not NaN
                {
                    // Find 3D position respect to rgb frame:
                    newPoint.z = depthValue;
                    newPoint.x = ((float) j); ///depthImage.cols; //  (j - rgbIntrinsicMatrix(0,2)) * newPoint.z * rgbFocalInvertedX;
                    newPoint.y = ((float) i); ///depthImage.rows; //(i - rgbIntrinsicMatrix(1,2)) * newPoint.z * rgbFocalInvertedY;
                    newPoint.r = 1.0; //rgbImage.at<cv::Vec3b>(i,j)[2];
                    newPoint.g = 0.0; //rgbImage.at<cv::Vec3b>(i,j)[1];
                    newPoint.b = 0.5; //rgbImage.at<cv::Vec3b>(i,j)[0];
                    cloud->push_back(newPoint);
                }
                else
                {
                    newPoint.z = std::numeric_limits<float>::quiet_NaN();
                    newPoint.x = std::numeric_limits<float>::quiet_NaN();
                    newPoint.y = std::numeric_limits<float>::quiet_NaN();
                    newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
                    newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
                    newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
                    cloud->push_back(newPoint);
                }
            }
        }
        */
        handleIncomingCloud(cloud);
    }

    void
    handleIncomingCloud (const CloudConstPtr& new_cloud)
    {
      CloudPtr temp_cloud (new Cloud);
      voxel_grid_filter_.setInputCloud (new_cloud);
      voxel_grid_filter_.filter (*temp_cloud);

      PointCloudBuffers::Ptr new_buffers = PointCloudBuffers::Ptr (new PointCloudBuffers);
      CopyPointCloudToBuffers (temp_cloud, *new_buffers);

      boost::mutex::scoped_lock lock (mutex_);
      filtered_cloud_ = temp_cloud;
      buffers_ = new_buffers;
    }

    PointCloudBuffers::Ptr
    getLatestBuffers ()
    {
      boost::mutex::scoped_lock lock (mutex_);
      return (buffers_);
    }

    CloudPtr
    getLatestPointCloud ()
    {
      boost::mutex::scoped_lock lock (mutex_);
      return (filtered_cloud_);
    }

    void
    run ()
    {
      
      //pcl::OpenNIGrabber grabber (device_id_);
      //boost::function<void (const CloudConstPtr&)> handler_function = boost::bind (&PCLMobileServer::handleIncomingCloud, this, _1);
      //grabber.registerCallback (handler_function);
      //grabber.start ();

      // wait for first cloud
      while (!getLatestPointCloud ())
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));

      viewer_.showCloud (getLatestPointCloud ());
      

      boost::asio::io_service io_service;
      tcp::endpoint endpoint (tcp::v4 (), static_cast<unsigned short> (port_));
      tcp::acceptor acceptor (io_service, endpoint);
      tcp::socket socket (io_service);

      std::cout << "Listening on port " << port_ << "..." << std::endl;
      acceptor.accept (socket);

      std::cout << "Client connected." << std::endl;

      double start_time = pcl::getTime ();
      int counter = 0;

      while (!viewer_.wasStopped ())
      {

        // wait for client
        unsigned int nr_points = 0;
        boost::asio::read (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

        PointCloudBuffers::Ptr buffers_to_send = getLatestBuffers ();

        nr_points = static_cast<unsigned int> (buffers_to_send->points.size()/3);
        boost::asio::write (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

        if (nr_points)
        {
          boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->points.front(), nr_points * 3 * sizeof (short)));
          boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->rgb.front(), nr_points * 3 * sizeof (unsigned char)));
        }

        counter++;

        double new_time = pcl::getTime ();
        double elapsed_time = new_time - start_time;
        if (elapsed_time > 1.0)
        {
          double frames_per_second = counter / elapsed_time;
          start_time = new_time;
          counter = 0;
          std::cout << "fps: " << frames_per_second << std::endl;
        }

        viewer_.showCloud (getLatestPointCloud ());
      }

      //grabber.stop ();
    }

};

