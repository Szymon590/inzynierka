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

Mat img1depth;
Mat img2depth;

void getPoint(unsigned int u, unsigned int v, float_type depth , Eigen::Vector3d& point3D){

    /// Stworzenie zmiennych focal
    double focalLength_fu = 525;//537.58330;
    double focalLength_fv = 525;//537.08866;
    double focalAxis_cu = 319.5;//314.54942;
    double focalAxis_cv = 239.5;//238.36211;

    ///Utworznie modelu PHCP
    PHCPModel << 1/focalLength_fu,0,-focalAxis_cu/focalLength_fu,
                  0,1/focalLength_fv, -focalAxis_cv/focalLength_fv,
                  0,0,1;

    Eigen::Vector3d point(u, v, 1);
    point3D = depth*PHCPModel*point;
}


void CallBackFunc(int event, int x, int y, int flags, void* ptr)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {

          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          Point*P = (Point*)ptr;
          if (img1depth.at<uint16_t>(y,x)*0.001<6&&img2depth.at<uint16_t>(y,x)*0.001<6){
            P->x = x;
            P->y = y;
            Eigen::Vector3d point3d;
            getPoint(x, y, img2depth.at<uint16_t>(y,x)*0.001, point3d);
            std::cout << "p3d: " << point3d(0) << ", " << point3d(1) << ", " << point3d(2) << "\n";
            cout << "New point P: " << P->x << "," << P->y << endl;
          }
     }
     else if  ( event == EVENT_RBUTTONDOWN ){

         std::cout << img1depth.at<uint16_t>(y,x)*0.001 << " 1\n";
         std::cout << img2depth.at<uint16_t>(y,x)*0.001 << " 2\n";

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
    Point P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16,P17,P18,P19,P20;
    unsigned int px1, py1,px2,py2,px3,px4,px5,px6,px7,px8,py3,py4,py5,py6,py7,py8,px9,px10,py9,py10;
    unsigned int px11, py11,px12,py12,px13,px14,px15,px16,px17,px18,py13,py14,py15,py16,py17,py18,px19,px20,py19,py20;

    ///Wczytuje obraz:
    Mat img1rgb   = imread("/home/szymon/Pulpit/trasy/trasa 2/rgb_00004.png");
    img1depth = imread("/home/szymon/Pulpit/trasy/trasa 2/depth_00004.png",CV_LOAD_IMAGE_ANYDEPTH);

    Mat img2rgb   = imread("/home/szymon/Pulpit/trasy/trasa 2/rgb_00001.png");
    img2depth = imread("/home/szymon/Pulpit/trasy/trasa 2/depth_00001.png",CV_LOAD_IMAGE_ANYDEPTH);

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
        namedWindow("My Window 9", 1);
        namedWindow("My Window 10", 1);
        namedWindow("My Window 11", 1);
        namedWindow("My Window 12", 1);
        namedWindow("My Window 13", 1);
        namedWindow("My Window 14", 1);
        namedWindow("My Window 15", 1);
        namedWindow("My Window 16", 1);
     //   namedWindow("My Window 17", 1);
     //   namedWindow("My Window 18", 1);
     //   namedWindow("My Window 19", 1);
     //   namedWindow("My Window 20", 1);

        ///funkcja zbierająca informacje z myszy: void setMouseCallback(const string& winname, MouseCallback onMouse, void* userdata = 0)
        setMouseCallback("My Window 1", CallBackFunc, &P1);
        setMouseCallback("My Window 2", CallBackFunc, &P2);


        setMouseCallback("My Window 3", CallBackFunc, &P3);
        setMouseCallback("My Window 4", CallBackFunc, &P4);


        setMouseCallback("My Window 5", CallBackFunc, &P5);
        setMouseCallback("My Window 6", CallBackFunc, &P6);


        setMouseCallback("My Window 7", CallBackFunc, &P7);
        setMouseCallback("My Window 8", CallBackFunc, &P8);

        setMouseCallback("My Window 9", CallBackFunc, &P9);
        setMouseCallback("My Window 10", CallBackFunc, &P10);

        setMouseCallback("My Window 11", CallBackFunc, &P11);
        setMouseCallback("My Window 12", CallBackFunc, &P12);


        setMouseCallback("My Window 13", CallBackFunc, &P13);
        setMouseCallback("My Window 14", CallBackFunc, &P14);


        setMouseCallback("My Window 15", CallBackFunc, &P15);
        setMouseCallback("My Window 16", CallBackFunc, &P16);


     //   setMouseCallback("My Window 17", CallBackFunc, &P17);
     //   setMouseCallback("My Window 18", CallBackFunc, &P18);

      //  setMouseCallback("My Window 19", CallBackFunc, &P19);
    //    setMouseCallback("My Window 20", CallBackFunc, &P20);

        ///pokazuje wczytany obraz w oknie "My window"
     //   imshow("My Window 20", img2rgb);
     //   imshow("My Window 19", img1rgb);
     //   imshow("My Window 18", img2rgb);
     //   imshow("My Window 17", img1rgb);
        imshow("My Window 16", img2rgb);
        imshow("My Window 15", img1rgb);
        imshow("My Window 14", img2rgb);
        imshow("My Window 13", img1rgb);
        imshow("My Window 12", img2rgb);
        imshow("My Window 11", img1rgb);
        imshow("My Window 10", img2rgb);
        imshow("My Window 9", img1rgb);
        imshow("My Window 8", img2rgb);
        imshow("My Window 7", img1rgb);
        imshow("My Window 6", img2rgb);
        imshow("My Window 5", img1rgb);
        imshow("My Window 4", img2rgb);
        imshow("My Window 3", img1rgb);
        imshow("My Window 2", img2rgb);
        imshow("My Window 1", img1rgb);


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

        px9 = P9.x;
        py9 = P9.y;
        px10 = P10.x;
        py10 = P10.y;

        px11 = P11.x;
        py11 = P11.y;
        px12 = P12.x;
        py12 = P12.y;

        px13 = P13.x;
        py13 = P13.y;
        px14 = P14.x;
        py14 = P14.y;

        px15 = P15.x;
        py15 = P15.y;
        px16 = P16.x;
        py16 = P16.y;

  /*      px17 = P17.x;
        py17 = P17.y;
        px18 = P18.x;
        py18 = P18.y;

        px19 = P19.x;
        py19 = P19.y;
        px20 = P20.x;
        py20 = P20.y;*/



        ///Pobieranie danych z obrazu skończone

        Eigen::Vector3d point;
        Eigen::Vector3d point2,point3,point1,point4,point5,point6,point7,point8,point9,point10;
        Eigen::Vector3d point12,point13,point11,point14,point15,point16,point17,point18,point19,point20;

        ///Utworzenie zmiennych dephtM( określające odległość od kamery ? )
        float_type depthM1 = float_type (img1depth.at<uint16_t>(py1,px1)*0.001);
        float_type depthM2 = float_type (img2depth.at<uint16_t>(py2,px2)*0.001);
        float_type depthM3 = float_type (img1depth.at<uint16_t>(py3,px3)*0.001);
        float_type depthM4 = float_type (img2depth.at<uint16_t>(py4,px4)*0.001);
        float_type depthM5 = float_type (img1depth.at<uint16_t>(py5,px5)*0.001);
        float_type depthM6 = float_type (img2depth.at<uint16_t>(py6,px6)*0.001);
        float_type depthM7 = float_type (img1depth.at<uint16_t>(py7,px7)*0.001);
        float_type depthM8 = float_type (img2depth.at<uint16_t>(py8,px8)*0.001);
        float_type depthM9 = float_type (img1depth.at<uint16_t>(py9,px9)*0.001);
        float_type depthM10 = float_type (img2depth.at<uint16_t>(py10,px10)*0.001);
        float_type depthM11 = float_type (img1depth.at<uint16_t>(py11,px11)*0.001);
        float_type depthM12 = float_type (img2depth.at<uint16_t>(py12,px12)*0.001);
        float_type depthM13 = float_type (img1depth.at<uint16_t>(py13,px13)*0.001);
        float_type depthM14 = float_type (img2depth.at<uint16_t>(py14,px14)*0.001);
        float_type depthM15 = float_type (img1depth.at<uint16_t>(py15,px15)*0.001);
        float_type depthM16 = float_type (img2depth.at<uint16_t>(py16,px16)*0.001);
     /*  float_type depthM17 = float_type (img1depth.at<uint16_t>(py17,px17)*0.001);
        float_type depthM18 = float_type (img2depth.at<uint16_t>(py18,px18)*0.001);
        float_type depthM19 = float_type (img1depth.at<uint16_t>(py19,px19)*0.001);
        float_type depthM20 = float_type (img2depth.at<uint16_t>(py20,px20)*0.001);*/


        cout <<"\npierwszy: "<< px1 << ", " << py1 << ", " <<depthM1 << endl;
        cout <<"drogi: "   << px2 << ", " << py2 << ", " <<depthM2 << endl;
        cout <<"pierwszy: "<< px3 << ", " << py3 << ", " <<depthM3 << endl;
        cout <<"drogi: "   << px4 << ", " << py4 << ", " <<depthM4 << endl;
        cout <<"pierwszy: "<< px5 << ", " << py5 << ", " <<depthM5 << endl;
        cout <<"drogi: "   << px6 << ", " << py6 << ", " <<depthM6 << endl;
        cout <<"pierwszy: "<< px7 << ", " << py7 << ", " <<depthM7 << endl;
        cout <<"drogi: "   << px8 << ", " << py8 << ", " <<depthM8 << endl;
        cout <<"pierwszy: "<< px9 << ", " << py9 << ", " <<depthM9 << endl;
        cout <<"drogi: "   << px10 << ", " << py10 << ", " <<depthM10 << endl<<endl;





        ///Pobieranie punktów
        getPoint(px1, py1, depthM1, point1);
        getPoint(px2, py2, depthM2, point2);
        getPoint(px3, py3, depthM3, point3);
        getPoint(px4, py4, depthM4, point4);
        getPoint(px5, py5, depthM5, point5);
        getPoint(px6, py6, depthM6, point6);
        getPoint(px7, py7, depthM7, point7);
        getPoint(px8, py8, depthM8, point8);
        getPoint(px9, py9, depthM9, point9);
        getPoint(px10, py10, depthM10, point10);
        getPoint(px11, py11, depthM11, point11);
        getPoint(px12, py12, depthM12, point12);
        getPoint(px13, py13, depthM13, point13);
        getPoint(px14, py14, depthM14, point14);
        getPoint(px15, py15, depthM15, point15);
        getPoint(px16, py16, depthM16, point16);
       /* getPoint(px17, py17, depthM17, point17);
        getPoint(px18, py18, depthM18, point18);
        getPoint(px19, py19, depthM19, point19);
        getPoint(px20, py20, depthM20, point20);*/


        ///Wyświetlanie
        cout << point1(0) << ", " << point1(1) << ", " << point1(2) <<endl;
        cout << point2(0) << ", " << point2(1) << ", " << point2(2) <<endl;
        cout << point3(0) << ", " << point3(1) << ", " << point3(2) <<endl;
        cout << point4(0) << ", " << point4(1) << ", " << point4(2) <<endl;
        cout << point5(0) << ", " << point5(1) << ", " << point5(2) <<endl;
        cout << point6(0) << ", " << point6(1) << ", " << point6(2) <<endl;
        cout << point7(0) << ", " << point7(1) << ", " << point7(2) <<endl;
        cout << point8(0) << ", " << point8(1) << ", " << point8(2) <<endl;
        cout << point9(0) << ", " << point9(1) << ", " << point9(2) <<endl;
        cout << point10(0) << ", " << point10(1) << ", " << point10(2) <<endl<<endl;

        ///Przekazanie punktów do umeyamy
        Vector3d a1(point1(0),point1(1),point1(2));
        Vector3d a2(point3(0),point3(1),point3(2));
        Vector3d a3(point5(0),point5(1),point5(2));
        Vector3d a4(point7(0),point7(1),point7(2));
        Vector3d a5(point9(0),point9(1),point9(2));
        Vector3d a6(point11(0),point11(1),point11(2));
        Vector3d a7(point13(0),point13(1),point13(2));
        Vector3d a8(point15(0),point15(1),point15(2));
     //   Vector3d a9(point17(0),point17(1),point17(2));
     //  Vector3d a10(point19(0),point19(1),point19(2));

        Vector3d b1(point2(0),point2(1),point2(2));
        Vector3d b2(point4(0),point4(1),point4(2));
        Vector3d b3(point6(0),point6(1),point6(2));
        Vector3d b4(point8(0),point8(1),point8(2));
        Vector3d b5(point10(0),point10(1),point10(2));
        Vector3d b6(point12(0),point12(1),point12(2));
        Vector3d b7(point14(0),point14(1),point14(2));
        Vector3d b8(point16(0),point16(1),point16(2));
     //   Vector3d b9(point18(0),point18(1),point18(2));
       // Vector3d b10(point20(0),point20(1),point20(2));

        Matrix<double,3,8> start,end;
        start.col(0)=a1;
        start.col(1)=a2;
        start.col(2)=a3;
        start.col(3)=a4;
        start.col(4)=a5;
        start.col(5)=a6;
        start.col(6)=a7;
        start.col(7)=a8;
       // start.col(8)=a9;
      //  start.col(9)=a10;


        end.col(0)=b1;
        end.col(1)=b2;
        end.col(2)=b3;
        end.col(3)=b4;
        end.col(4)=b5;
        end.col(5)=b6;
        end.col(6)=b7;
        end.col(7)=b8;
      //  end.col(8)=b9;
      //  end.col(9)=b10;

        cout << Eigen::umeyama(start,end,true) << endl<<endl;




        return 0;
}
