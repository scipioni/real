#include <iostream>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/packet_pipeline.h>
#include <unistd.h>
#include <stdio.h>

#include "processor/processor.hpp"

#define KMAX 4500.0

//using namespace std;

bool protonect_shutdown = false;
//PCLMobileServer<pcl::PointXYZRGBA> *server;

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

/*
*/

/*
   void ThreadServer()
   {
   int port = 11111;
   float leaf_x = 0.01f, leaf_y = 0.01f, leaf_z = 0.01f;
   server = new PCLMobileServer<pcl::PointXYZRGBA>("", port, leaf_x, leaf_y, leaf_z);

   try
   {
   server->run();
   }
   catch(boost::thread_interrupted&)
   {
   cout << "Thread is stopped" << endl;
   return;
   }
   }
   */

int main(int argc, char *argv[])
{
    int opt;
    float depth_min=600/KMAX;
    float depth_max=1500/KMAX;
    int levels=0;
    char *imagetest;
    int testmode = 0;
    int autopilot = 0;
    bool cpupacket = false;
    float scale = 1.0;
    while ((opt = getopt(argc, argv, "f:m:M:l:az:c")) != -1) {
        switch (opt) {
            case 'a':
                autopilot = 1;
                break;
            case 'c':
            	cpupacket = true;
            	break;
            case 'z':
                scale = atof(optarg); break;
            case 'f':
                imagetest = optarg; testmode = 1; break;
            case 'm':
                depth_min = atof(optarg); break;
            case 'M':
                depth_max = atof(optarg); break;
            case 'l':
                levels = atoi(optarg); break;
            default: /* '?' */
            	std::cerr <<  "Usage: " << argv[0] << " [-a] [-z zoom] [-l levels] [-m depth_min] [-M depth_max] [-f image] [-c]" << std::endl;
            	std::cerr <<  "a: autopilot" << std::endl;
            	std::cerr <<  "c: cpupacket" << std::endl;
            	std::exit(EXIT_FAILURE);
        }
    }


    Processor *processor = new Processor();

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
    //libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;

    cv::Mat mat;
    if (testmode) {
        cv::FileStorage storage(imagetest, cv::FileStorage::READ);
        storage["img"] >> mat;
        storage.release();
    } else {
    	if (cpupacket) {
    		dev = freenect2.openDefaultDevice(new libfreenect2::CpuPacketPipeline());
    	} else {
    		dev = freenect2.openDefaultDevice();
    		//dev = freenect2.openDefaultDevice(new libfreenect2::OpenCLPacketPipeline());
    	}
        if(dev == 0)
        {
        	std::cout << "no device connected or failure opening the default one!" << std::endl;
            return -1;
        }

        //dev->setColorFrameListener(&listener);
        dev->setIrAndDepthFrameListener(&listener);
        dev->start();

        std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
        std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    }

    std::signal(SIGINT,sigint_handler);
    protonect_shutdown = false;



    //boost::thread threadServer(&ThreadServer);    

    double time_elapsed=0;
    while(!protonect_shutdown)
    {
        processor->resetTimer();
        cv::Point maxLoc; 

        if (! testmode) {
            listener.waitForNewFrame(frames);
            //libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
            processor->setDepth(depth->width, depth->height, depth->data, depth_min, depth_max);

            //cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
            //cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
            //cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);
            mat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);

        }

        if (autopilot)
             processor->adjustDepth();

        processor->network();

        //server->setMat(mat, depth_min, depth_max);


        //if (scale != 1) cv::resize(img_color, img_color, cv::Size(scale*img_color.cols,scale*img_color.rows));

        int key = cv::waitKey(1);
        if (key > 0) {
            key = key & 0xFF;
            std::cout << "key: " << key << std::endl;
            switch (key) {
                case 27: protonect_shutdown = true; break;
                case 82: depth_max += 50/KMAX; break;
                case 84: depth_max -= 50/KMAX; break;
                case 83: depth_min += 50/KMAX; break;
                case 81: depth_min -= 50/KMAX; break;
                case 's':
                		 std::cout << "Saving depth.yml" << std::endl;
                         cv::FileStorage storage("depth.yml", cv::FileStorage::WRITE);
                         storage << "img" << mat;
                         storage.release();  
                         break;
            }
            std::cout << "depth_min=" << depth_min << " depth_max=" << depth_max << std::endl;
        }

        if (! testmode) 
            listener.release(frames);
        //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));

        //time_elapsed = processor->getTimer();
    }

    //cout << "main: thread interrupt" << endl;
    //threadServer.interrupt();    
    //threadServer.join();    
    //cout << "main: thread ended" << endl;

    if (! testmode) {
        dev->stop();
        dev->close();
    }

    delete processor;

    return 0;
}
