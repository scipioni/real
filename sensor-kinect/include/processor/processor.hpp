#include <opencv2/opencv.hpp>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace pcl;

class Processor
{
    cv::Mat depth;
    size_t depth_w;
    size_t depth_h;
    float depth_min, depth_max;

    cv::Point maxLoc;

    unsigned counter;
    unsigned getTickCount();

    Network yarp;
    //BufferedPort<ImageOf<PixelBgr> > colorPort;  // make a port for reading images
    Port colorPort;

    void initYarp();

public:
    Processor();
    ~Processor();

    float getMemoryUsageMB();
    void resetTimer();
    unsigned getTimer();

    void setDepth(size_t w, size_t h, unsigned char data[], float depth_min, float depth_max);

    void adjustDepth();
    void network();
};

