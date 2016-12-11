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


string int2str(int i)
{
    stringstream ss;
    string temp;
    ss << i;
    ss >> temp;
    return temp;
}

int main( int argc, char** argv )
{
    int n=8;//zawsze parzyste musi byc !
    Point P[n];
    unsigned int px[n];
    unsigned int py[n];
    Eigen::Vector3d point[n];
    float_type depth[n];
  //  Vector3d b[n];
   // Matrix<double,3,4> start,end;




    ///Wczytuje obraz:
    Mat img1rgb   = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/rgb_00043.png");
    img1depth = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/depth_00043.png",CV_LOAD_IMAGE_ANYDEPTH);

    Mat img2rgb   = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/rgb_00044.png");
    img2depth = imread("/home/szymon/Pulpit/Programik/dzialania wojenne/depth_00044.png",CV_LOAD_IMAGE_ANYDEPTH);

    ///Jeśli obraz nie może zostać wczytany:
    if ( img1rgb.empty() || img1depth.empty() || img2rgb.empty() || img1depth.empty() )
    {
         cout << "Error loading the image" << endl;
         return -1;
    }



     for(int i=0;i<n;i++)
     {
         string nazwa = "My Window " + int2str(i);//tworzymy nazwy okien
         namedWindow(nazwa , 1);//tworzymy okna
         if(i%2 == 0)
         {
            imshow(nazwa,img1rgb);//wyświetlanie okien
         }
         else if(i%2 == 1)
         {
            imshow(nazwa,img2rgb);
         }
     }

     for(int i=0;i<n;i++)
     {
         string nazwa = "My Window " + int2str(i);
         setMouseCallback(nazwa,CallBackFunc,&P[i]);//klikanko
     }


     ///0 - czekanie w nieskończoność
     waitKey(0);

     for(int i=0;i<n;i++)
     {
         px[i]=P[i].x;
         py[i]=P[i].y;// punkciki, w sumie do wywalenia
         depth[i] = float_type (img1depth.at<uint16_t>(py[i],px[i])*0.001);//głębia
         getPoint(px[i], py[i], depth[i], point[i]);// konwersja z pkt 2d do 3d
     }

     cout<<endl;
     for(int i=0;i<n;i++)
     {
         cout <<"x: "<< px[i] << ",y: " << py[i] << ",depth: " <<depth[i] << endl;
         cout <<"x: " << point[i](0) << ",y: " << point[i](1) << ",z: " << point[i](2) <<endl<<endl;
     }
     cout<<endl;


  /*   // VectorXd a[n];
     Matrix<double,3,1> a[n];


     for(int i=0;i<n;i++)
     {
      a[i](0)=point[i](0);
      a[i](1)=point[i](1);
      a[i](2)=point[i](2);
      }
     }


    /* for(int i=0;i<n;i++)
     {
         if(i%2 == 0)
         {
            start.col(s)=a[i];
            s++;
         }
         else if(i%2 == 1)
         {
            end.col(e)=b[i];
            e++;
         }
     }*/



     ///Przekazanie punktów do umeyamy
     Vector3d b1(point[1](0),point[1](1),point[1](2));
     Vector3d b2(point[3](0),point[3](1),point[3](2));
     Vector3d b3(point[5](0),point[5](1),point[5](2));
     Vector3d b4(point[7](0),point[7](1),point[7](2));
   //  Vector3d a5(point9(0),point9(1),point9(2));

     Vector3d a2(point[2](0),point[2](1),point[2](2));
     Vector3d a3(point[4](0),point[4](1),point[4](2));
     Vector3d a4(point[6](0),point[6](1),point[6](2));
     Vector3d a1(point[0](0),point[0](1),point[0](2));
    // Vector3d b5(point10(0),point10(1),point10(2));

     Matrix<double,3,4> start,end;
     start.col(0)=a1;
     start.col(1)=a2;
     start.col(2)=a3;
     start.col(3)=a4;
    // start.col(4)=a5;


     end.col(0)=b1;
     end.col(1)=b2;
     end.col(2)=b3;
     end.col(3)=b4;
    // end.col(4)=b5;
   cout << Eigen::umeyama(start,end,true) << endl<<endl;
    int st=0;
     int en=0;
     for(int i=0;i<n;i++)
     {
        Vector3d a(point[i](0),point[i](1),point[i](1));
        if(i%2 == 0)
        {
            start.col(st)=a;
            st++;
        }
        else if(i%2 == 1)
        {
            end.col(en)=a;
            en++;
        }
     }

     cout << Eigen::umeyama(start,end,true) << endl<<endl;



        return 0;
}
