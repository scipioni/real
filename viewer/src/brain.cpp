#include <iostream>
#include "brain.hpp"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

#define COLOR_MYPORT "/viewer/color"
#define COLOR_PORT "/real/sensor/color"

Brain::Brain()  {
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new	pcl::PointCloud<pcl::PointXYZ>);

	imagePort.open(COLOR_MYPORT);  // give the port a name
	yarp.connect(COLOR_PORT, imagePort.getName());

    cout << "Brain initialized" << endl;

}

Brain::~Brain()  {
    cout << "Brain destroyed" << endl;
}


bool Brain::readDepth() {
	/*
	 * depth data are in range [0..4500]
	 */
	ImageOf<PixelFloat> *yarpImage = imagePort.read();  // read an image
	if (yarpImage != NULL) { // check we actually got something
		//printf("We got an image of size %dx%d\n", yarpImage->width(), yarpImage->height());
		depth = cv::Mat(static_cast<IplImage*>(yarpImage->getIplImage()));
		img_color_need_rebuild = cloud_need_rebuild = true;
		return true;
	}
	return false;
}

cv::Mat& Brain::getColor() {
	if (img_color_need_rebuild)
		convertDepthToColors();
	return img_color;
}

void Brain::convertDepthToColors() {
	img_color = cv::Mat(depth.rows, depth.cols, CV_8UC4);

    unsigned char *out = (unsigned char*)(img_color.data);
    register int k, g;
    register float depth_value;

    for(int i=0; i<depth.rows; i++) {
        for(int j=0; j<depth.cols; j++) {  // colonna
            k=depth.cols*i+j;
            //depth_value = input[k];
            depth_value = depth.at<float>(i,j); // input[k];

            if (depth_value < depth_min) { // bianco prima del target
                out[4*k] = 0xFF;
                out[4*k+1] = 0xFF;
                out[4*k+2] = 0xFF;
            } else if (depth_value > depth_max) { // bianco dopo il target
                out[4*k] = 0xFF;
                out[4*k+1] = 0xFF;
                out[4*k+2] = 0xFF;
            } else {
                g = 255 - ((depth_value-depth_min)*255)/(depth_max-depth_min);
                if (g <= 0x7F) {
                    out[4*k+2] = g; // R
                    out[4*k+1] = 0x7F+g; // G
                    out[4*k] = 0xFF-g*2; // B
                } else {
                    out[4*k+2] = g; // R
                    out[4*k+1] = 0xFF-(g-0x7F)*2; // G
                    out[4*k] = 0; // B
                }
            }
        }
    }
    img_color_need_rebuild = false;
}


void Brain::convertDepthToCloud() {
		if (cloud->width != depth.cols) {
			cloud->width = depth.cols;
			cloud->height = depth.rows;
			cloud->resize(depth.cols * depth.rows);
		}

		register int u, v, idx=0;
		register float constant = 1.0f / 525, d;

		for(int u=0; u<depth.rows; u++) {
	        for(int v=0; v<depth.cols; v++, idx++) {  // colonna

	        	pcl::PointXYZ& pt = cloud->points[idx];

	        	d = depth.at<float>(u,v);
	            if (d >= depth_min && d <= depth_max) {
	            	pt.z = d-depth_min; // z si trova tra 0 e (depth_max-depth_min)
		        	pt.x = static_cast<float> (v);
		            pt.y = depth.rows - static_cast<float> (u);

	            } else  {
	            	pt.x = pt.y = pt.z = bad_point;
	            }
	        }
		}
		cloud_need_rebuild = false;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr& Brain::getCloud() {
	if (cloud_need_rebuild)
		convertDepthToCloud();
	return cloud;
}

/*
cv::Mat& Processor::contours(int levels) {
	if (levels <= 0)
		return img_color;

    cv::Mat gray, bw;
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;

    cv::cvtColor(img_color, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size( 3, 3 ), 0, 0);

    for (int level=1; level<=levels; level++) {
        cv::threshold(gray, bw, level*0xFF/(levels+1), 255, CV_THRESH_BINARY);

        cv::findContours( bw, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        for( int i = 0; i< contours.size(); i++ )
        {
            //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            if (cv::contourArea(contours[i]) > 100) {
                cv::drawContours( img_color, contours, i, cv::Scalar(0,0,0), 1, 8, hierarchy, 0, cv::Point() );
            }
        }
    }
    return img_color;
}
*/
