#include <opencv2/opencv.hpp>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>
#include <limits>

using namespace yarp::os;
using namespace yarp::sig;
using namespace pcl;

class Brain
{
	Network yarp;

	float depth_min=600;
	float depth_max=1500;

	const float bad_point = std::numeric_limits<float>::quiet_NaN();

	BufferedPort<ImageOf<PixelFloat> > imagePort; // make a port for reading images
	cv::Mat depth;
	cv::Mat img_color;
    bool img_color_need_rebuild=true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    bool cloud_need_rebuild=true;

	void convertDepthToColors();
	void convertDepthToCloud();

public:
	Brain();
	~Brain();
	bool readDepth();
	cv::Mat& getColor();
	pcl::PointCloud<pcl::PointXYZ>::Ptr& getCloud();
};
