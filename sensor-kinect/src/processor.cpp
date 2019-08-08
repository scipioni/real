#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <iterator>
#include "processor/processor.hpp"
#include <yarp/os/all.h>
#include <stdio.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/* ricreare il file PointCloudYarp.h:
 * editare PointCloudYarp.msg
 * lanciare yarpidl_rosmsg PointCloudYarp.msg
 * */
#include "processor/PointCloudYarp.h"


#define KMAX 4500.0
#define CLOUD_PORT "/real/sensor/cloud"
#define COLOR_PORT "/real/sensor/color"

using namespace std;
using namespace yarp::os;

int parseLine(char* line){
    int i = strlen(line);
    while (*line < '0' || *line > '9') line++;
    line[i-3] = '\0';
    i = atoi(line);
    return i;
}

Processor::Processor()  {
    cout << "Processor initialized" << endl;
    depth_w=0;
    depth_h=0;
    initYarp();

}

void Processor::initYarp() {
    colorPort.open(COLOR_PORT);  // give the port a name
}

Processor::~Processor()  {
    cout << "Processor destroyed" << endl;
}

float Processor::getMemoryUsageMB() { 
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmSize:", 7) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result/1000.0;
}

unsigned Processor::getTickCount()
{
    struct timeval tv;
    if(gettimeofday(&tv, NULL) != 0)
        return 0;

    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

void Processor::resetTimer()  {
    counter = getTickCount();
}

unsigned Processor::getTimer()  {
    return getTickCount() - counter;
}

void Processor::setDepth(size_t w, size_t h, unsigned char data[],	float depth_min, float depth_max) {
	this->depth_min = depth_min;
	this->depth_max = depth_max;

	depth = cv::Mat(h, w, CV_32FC1, data); // / KMAX;
	if (depth_w == 0) {
		depth_w = w;
		depth_h = h;
	}
}



void Processor::adjustDepth() {
    double minVal, maxVal;
    cv::Point minLoc;

    cv::Mat mirino = depth(cv::Rect(depth.cols/5, depth.rows/5, 4*depth.cols/5, 4*depth.rows/5));
    cv::GaussianBlur(mirino, mirino, cv::Size( 3, 3 ), 0, 0);
    cv::Scalar media = cv::mean(mirino);
    minMaxLoc(mirino, &minVal, &maxVal, &minLoc, &maxLoc); // minVal qui è sempre 0
    maxLoc.x += depth.cols/5;
    maxLoc.y += depth.rows/5;
    depth_max = maxVal;
    minVal = 2*media[0]-maxVal; // la media è nel mezzo ;-)
    if (minVal > 500/KMAX) depth_min = minVal;
}

void Processor::network() {
    ImageOf<PixelFloat> yarpImage;
    yarpImage.setExternal(depth.data, depth.cols, depth.rows);
    colorPort.write(yarpImage);
}

