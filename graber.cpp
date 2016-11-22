#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "include/Grabber/xtionAsus.h"
#include <Eigen/Dense>

using namespace cv;
using namespace std;
using namespace Eigen;

float_type focalLength[2];
float_type focalAxis[2];

//pin-hole camera projection model

Mat33 PHCPModel;


void getPoint(unsigned int u, unsigned int v, float_type depth , Eigen::Vector3d& point3D){
    Eigen::Vector3d point(u, v, 1);
    point3D = depth*PHCPModel*point;
}


void CallBackFunc(int event, int x, int y, int flags, void* ptr)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          Point*P = (Point*)ptr;
          P->x = x;
          P->y = y;
          cout << "New point P: " << P->x << "," << P->y << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

     }
     else if ( event == EVENT_MOUSEMOVE )
     {
         // cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}

int main( int argc, char** argv )
{
    Point P1,P2,P3,P4,P5,P6,P7,P8;
    unsigned int px1, py1,px2,py2,px3,px4,px5,px6,px7,px8,py3,py4,py5,py6,py7,py8;

    ///Wczytuje obraz:
    Mat img1rgb   = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/rgb_00043.png");
    Mat img1depth = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/depth_00043.png");

    Mat img2rgb   = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/rgb_00044.png");
    Mat img2depth = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/depth_00044.png");

        ///Jeśli obraz nie może zostać wczytany:
        if ( img1rgb.empty() || img1depth.empty() || img2rgb.empty() || img1depth.empty() )
        {
             cout << "Error loading the image" << endl;
             return -1;
        }

        ///Tworzy okno:
        namedWindow("My Window 1", 1);
        namedWindow("My Window 2", 1);
        namedWindow("My Window 3", 1);
        namedWindow("My Window 4", 1);
        namedWindow("My Window 5", 1);
        namedWindow("My Window 6", 1);
        namedWindow("My Window 7", 1);
        namedWindow("My Window 8", 1);

        ///funkcja zbierająca informacje z myszy: void setMouseCallback(const string& winname, MouseCallback onMouse, void* userdata = 0)
        setMouseCallback("My Window 1", CallBackFunc, &P1);
        setMouseCallback("My Window 2", CallBackFunc, &P2);


        setMouseCallback("My Window 3", CallBackFunc, &P3);
        setMouseCallback("My Window 4", CallBackFunc, &P4);


        setMouseCallback("My Window 5", CallBackFunc, &P5);
        setMouseCallback("My Window 6", CallBackFunc, &P6);


        setMouseCallback("My Window 7", CallBackFunc, &P7);
        setMouseCallback("My Window 8", CallBackFunc, &P8);


        ///pokazuje wczytany obraz w oknie "My window"
        imshow("My Window 1", img1rgb);
        imshow("My Window 2", img2rgb);
        imshow("My Window 3", img1rgb);
        imshow("My Window 4", img2rgb);
        imshow("My Window 5", img1rgb);
        imshow("My Window 6", img2rgb);
        imshow("My Window 7", img1rgb);
        imshow("My Window 8", img2rgb);

        ///0 - czekanie w nieskończoność
        waitKey(0);

        px1 = P1.x;
        py1 = P1.y;
        px2 = P2.x;
        py2 = P2.y;

        px3 = P3.x;
        py3 = P3.y;
        px4 = P4.x;
        py4 = P4.y;

        px5 = P5.x;
        py5 = P5.y;
        px6 = P6.x;
        py6 = P6.y;

        px7 = P7.x;
        py7 = P7.y;
        px8 = P8.x;
        py8 = P8.y;

        cout <<"\npierwszy: "<< px1 << ", " << py1 << endl;
        cout <<"drogi: "   << px2 << ", " << py2 << endl;
        cout <<"pierwszy: "<< px3 << ", " << py3 << endl;
        cout <<"drogi: "   << px4 << ", " << py4 << endl;
        cout <<"pierwszy: "<< px5 << ", " << py5 << endl;
        cout <<"drogi: "   << px6 << ", " << py6 << endl;
        cout <<"pierwszy: "<< px7 << ", " << py7 << endl;
        cout <<"drogi: "   << px8 << ", " << py8 << endl<<endl;

        ///Pobieranie danych z obrazu skończone

        Eigen::Vector3d point;
        Eigen::Vector3d point2,point3,point1,point4,point5,point6,point7,point8;

        ///Utworzenie zmiennych dephtM( określające odległość od kamery ? )
        float_type depthM1 = float_type (img1depth.at<uint16_t>(px1,py1)*0.001);
        float_type depthM2 = float_type (img2depth.at<uint16_t>(px2,py2)*0.001);
        float_type depthM3 = float_type (img1depth.at<uint16_t>(px3,py3)*0.001);
        float_type depthM4 = float_type (img2depth.at<uint16_t>(px4,py4)*0.001);
        float_type depthM5 = float_type (img1depth.at<uint16_t>(px5,py5)*0.001);
        float_type depthM6 = float_type (img2depth.at<uint16_t>(px6,py6)*0.001);
        float_type depthM7 = float_type (img1depth.at<uint16_t>(px7,py7)*0.001);
        float_type depthM8 = float_type (img2depth.at<uint16_t>(px8,py8)*0.001);


        /// Stworzenie zmiennych focal
        double focalLength_fu = 537.58330;
        double focalLength_fv = 537.08866;
        double focalAxis_cu = 314.54942;
        double focalAxis_cv = 238.36211;
        
        ///Utworznie modelu PHCP
        PHCPModel << 1/focalLength_fu,0,-focalAxis_cu/focalLength_fu,
                      0,1/focalLength_fv, -focalAxis_cv/focalLength_fv,
                      0,0,1;



        ///Pobieranie punktów
        getPoint(py1, px1, depthM1, point1);
        getPoint(py2, px2, depthM2, point2);
        getPoint(py3, px3, depthM3, point3);
        getPoint(py4, px4, depthM4, point4);
        getPoint(py5, px5, depthM5, point5);
        getPoint(py6, px6, depthM6, point6);
        getPoint(py7, px7, depthM7, point7);
        getPoint(py8, px8, depthM8, point8);



        cout << point1(0) << ", " << point1(1) << ", " << point1(2) <<endl;
        cout << point2(0) << ", " << point2(1) << ", " << point2(2) <<endl;
        cout << point3(0) << ", " << point3(1) << ", " << point3(2) <<endl;
        cout << point4(0) << ", " << point4(1) << ", " << point4(2) <<endl;
        cout << point5(0) << ", " << point5(1) << ", " << point5(2) <<endl;
        cout << point6(0) << ", " << point6(1) << ", " << point6(2) <<endl;
        cout << point7(0) << ", " << point7(1) << ", " << point7(2) <<endl;
        cout << point8(0) << ", " << point8(1) << ", " << point8(2) <<endl<<endl;


        ///Przekazanie punktów do umeyamy
        Vector3d a1(point1(0),point1(1),point1(2));
        Vector3d a2(point3(0),point3(1),point3(2));
        Vector3d a3(point5(0),point5(1),point5(2));
        Vector3d a4(point7(0),point7(1),point7(2));

        Vector3d b1(point2(0),point2(1),point2(2));
        Vector3d b2(point4(0),point4(1),point4(2));
        Vector3d b3(point6(0),point6(1),point6(2));
        Vector3d b4(point8(0),point8(1),point8(2));

        Matrix<double,3,4> start,end;
        start.col(0)=a1;
        start.col(1)=a2;
        start.col(2)=a3;
        start.col(3)=a4;


        end.col(0)=b1;
        end.col(1)=b2;
        end.col(2)=b3;
        end.col(3)=b4;

        cout << Eigen::umeyama(start,end,true) << endl<<endl;


        return 0;
}
