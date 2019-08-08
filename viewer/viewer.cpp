#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include "src/brain.hpp"

#define COLORS_WINDOW "colors depth"
#define CLOUD_WINDOW "Cloud"
#define ID_CLOUD_1 "cloud1"
#define CAMERA_CFG "camera.cfg"

int main() {
	Brain *brain = new Brain();

	cv::namedWindow(COLORS_WINDOW, CV_WINDOW_AUTOSIZE);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(CLOUD_WINDOW));
	viewer->setBackgroundColor(1.0, 1.0, 1.0);
	viewer->addPointCloud<pcl::PointXYZ>(brain->getCloud(), ID_CLOUD_1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ID_CLOUD_1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 0.0f, ID_CLOUD_1);
	//viewer->addCoordinateSystem(1000.0); // x = red, y = gre, z = blu
	viewer->initCameraParameters();
	viewer->resetCameraViewpoint(ID_CLOUD_1);
	viewer->loadCameraParameters(CAMERA_CFG);

	while (1) {
		if (brain->readDepth()) {
			cv::imshow(COLORS_WINDOW, brain->getColor());
			unsigned char key = cv::waitKey(10);
			if (key == 27)
				break;
			else if (key == 's') {
				viewer->saveCameraParameters(CAMERA_CFG);
				cv::imwrite( "/tmp/test.jpg", matImage );
			} else if (key == 'r') {
				//viewer->removePointCloud("sample cloud");
			} else if (key == ' ') {
				//viewer->updatePointCloud<pcl::PointXYZ>(brain->getCloud(), ID_CLOUD_1);
			}
		}
		viewer->updatePointCloud<pcl::PointXYZ>(brain->getCloud(), ID_CLOUD_1);
		viewer->spinOnce (100);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	return 0;
}
