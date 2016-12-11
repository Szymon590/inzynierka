#include "include/Defs/tracker_defs.h"
#include "include/Grabber/genericCam.h"
//#include "Grabber/uEyeGrabber.h"
#include "include/Grabber/fileGrabber.h"
#include "include/Grabber/xtionAsus.h"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "Utilities/CLParser.h"
/*#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>*/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


int main(int argc, char * argv[])
{
    try {
      //  CLParser cmd_line(argc,argv,true);
      // if (cmd_line.get_arg("-h").size())
      //     std::cout << "To run type: ./grabberConv -i depthImage.png -o outputCloud.pcd\n";


       /* if (cmd_line.get_arg("-i").length()!=0){
            inputFile = cmd_line.get_arg("-i");
            if (cmd_line.get_arg("-o").length()!=0){
                outputFile = cmd_line.get_arg("-o");
            }
            else {
                std::cout << "No output file specified (-i fileneme)\n";
                return 0;
            }
        }
        else {
            std::cout << "No input file specified (-i fileneme)\n";
            return 0;
        }*/
        std::string inputFile = "/home/szymon/Pulpit/Programik/dzialania wojenne/depth_00045.png";
        std::string inputFile1 = "/home/szymon/Pulpit/Programik/dzialania wojenne/rgb_00045.png";
        std::string outputFile = "nowynowy66.pcd";
        Mat img,img1;

       // tinyxml2::XMLDocument config;
        //config.LoadFile("KinectModel.xml");


        img = cv::imread(inputFile, CV_LOAD_IMAGE_ANYDEPTH );; // wczytywanie zdjecia depth
        img1 = cv::imread(inputFile1);

        AsusGrabber::UncertaintyModel asusModel("../resources/KinectModel.xml");

        PointCloud outCloud = asusModel.depth2cloudcolor(img,img1); // zapisywanie zdjecia depth do pcd

     //   cout<<endl<<outCloud.points[5].x<<" "<<outCloud.points[5].y<<" "<<outCloud.points[5].z<<endl;
     //   cout<<endl<<outCloud.points[1115].x<<" "<<outCloud.points[1115].y<<" "<<outCloud.points[1115].z<<endl;
       // cout<<endl<<outCloud.points[11115].x<<" "<<outCloud.points[11115].y<<" "<<outCloud.points[11115].z<<endl;

      //  if (config.ErrorID())
       //     std::cout << "unable to load Kinect config file.\n";

        //pcl::PointCloud<pcl::PointXYZRGBA> nowy = asusModel.depth2cloud(img); // zapisywanie zdjecia depth do pcd

        //uff ten pointcloud to tylko ten pclowy

        pcl::io::savePCDFile(outputFile, outCloud,1);
        std::cout << "Done.\n";
}
    catch(const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
   }
    return 0;

}
