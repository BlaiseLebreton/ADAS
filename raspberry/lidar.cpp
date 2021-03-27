#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <unistd.h>
#include <math.h>

using namespace std;
using namespace ydlidar;
using namespace cv;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

CYdLidar laser;
int WIDTH = 250;
int SCALE = WIDTH/(8.0*2.0);
Mat data = Mat::zeros(Size(WIDTH, WIDTH), CV_8UC3);
Mat plot_result;
int sx,sy,cx,cy,lx,rx,psid=0,type,dist;
float R,Rs,R_i,R_e,E=0.3,L=0.3,psi,Ro_m;
Point P_m;
int Lidar_Initialize(int DISPLAY) {
  std::string port;
  int ac;
  char* av[3];
  ydlidar::init(ac, av);

  std::map<std::string, std::string> ports = ydlidar::YDlidarDriver::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  }
  else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::ok()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }


  if (!ydlidar::ok()) {
    return 0;
  }

  laser.setSerialPort(port);
  laser.setSerialBaudrate(115200);
  laser.setFixedResolution(false);
  laser.setReversion(false);
  laser.setInverted(false);
  laser.setAutoReconnect(true);
  laser.setSingleChannel(true);
  laser.setLidarType(TYPE_TOF);
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);
  laser.setMinRange(0.1);
  laser.setMaxRange(8.0);
  laser.setScanFrequency(8.0);

  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  if (DISPLAY == 1) {
    // Window
    namedWindow("LIDAR Datas",  WINDOW_NORMAL);
    resizeWindow("LIDAR Datas", WIDTH, WIDTH);

    // Trackbar
    createTrackbar("Angle braquage", "LIDAR Datas", &psid,  90);
    setTrackbarMin("Angle braquage", "LIDAR Datas",        -90);
  }

  // Test get data
  bool hardError;
  LaserScan scan;
  if (laser.doProcessSimple(scan, hardError)) {
    printf("LIDAR OK\n");
  }
  else {
    printf("LIDAR NOK\n");
  }
  return 0;
}

float Lidar_CheckObstacles(int* cmd, int* pwr, int DISPLAY) {
  if (!ydlidar::ok()) {
    return 1;
  }

  bool hardError;
  LaserScan scan;
  bool IsObstacle = false;
  data = 0;

  // TODO: A calculer avec (*cmd)
  psi = psid;

  // Type ligne droite (1) ou arc de cercle (0)
  type = abs(psi) < 0.1 ? 1 : 0;

  if (type == 1) {
    lx = WIDTH/2 - L/2*SCALE;
    rx = WIDTH/2 + L/2*SCALE;
  }
  else {
    R = E/cos((90-psi)*2*M_PI/360.0); // calcul de rayon de la trajectoire en fonction de l'empattement et de rayon de braquage

    if (psi > 0) {
      R_i = abs(R - (L/2));
      R_e = abs(R + (L/2));
    }
    else {
      R_i = abs(R + (L/2));
      R_e = abs(R - (L/2));
    }

    cy = WIDTH/2;
    cx = WIDTH/2 - R*SCALE;
  }

  if (laser.doProcessSimple(scan, hardError)) {
    Ro_m = HUGE_VALF;
    P_m  = Point(WIDTH/2, WIDTH/2);
    for(int i = 0; i < scan.points.size(); i++) {
      IsObstacle = false;

      scan.points[i].range = scan.points[i].range / 4;
      sx = scan.points[i].range*SCALE*cos(scan.points[i].angle+M_PI/2.0) + WIDTH/2;
      sy = scan.points[i].range*SCALE*sin(scan.points[i].angle+M_PI/2.0) + WIDTH/2;

      // Green by default
      if (DISPLAY == 1)
        data.at<Vec3b>(sy, sx) = Vec3b(0,255,0);

      if (type == 1) {
        if (sx >= lx && sx <= rx) {
          if (DISPLAY == 1)
            data.at<Vec3b>(round(sy), round(sx)) = Vec3b(0,0,255);
          IsObstacle = true;
        }
      }
      else {
        Rs = sqrtf((sx-cx)*(sx-cx) + (sy-cy)*(sy-cy)) / SCALE;

        // Red if in trajectory
        if (Rs >= R_i && Rs <= R_e) {
          if (DISPLAY == 1)
            data.at<Vec3b>(round(sy), round(sx)) = Vec3b(0,0,255);
          IsObstacle = true;
        }
      }

      // Green if behind
      if (sy >= WIDTH/2) {
        if (DISPLAY == 1)
          data.at<Vec3b>(round(sy), round(sx)) = Vec3b(0,255,0);
        IsObstacle = false;
      }

      // Closest obstacle
      if (IsObstacle && scan.points[i].range > 0) {
        if (scan.points[i].range < Ro_m) {
          Ro_m = scan.points[i].range;
          P_m  = Point(round(sx), round(sy));
        }
      }

    }

    // Traitement obstacle le plus proche
    if (Ro_m <= 0.4){
      *pwr = 1500;
    }
    if (DISPLAY == 1)
      line(data, Point(WIDTH/2, WIDTH/2), P_m, Scalar(255,255,255), 1);
  }
  else {
    fprintf(stderr, "Failed to get Lidar Data\n");
    fflush(stderr);
  }
  if (DISPLAY == 1) {
    if (type == 1) {
      line(data, Point(WIDTH/2, WIDTH), Point(WIDTH/2, 0), Scalar(0,     0, 255), 1, LINE_8); // Red
      line(data, Point(lx ,     WIDTH), Point(lx,      0), Scalar(0,   255,   0), 1, LINE_8); // Green
      line(data, Point(rx,      WIDTH), Point(rx,      0), Scalar(255,   0,   0), 1, LINE_8); // Blue
    }
    else {
      circle(data, Point(cx, cy), abs(R  *SCALE), Scalar(0,     0, 255), 1, LINE_8); // Red
      circle(data, Point(cx, cy),     R_i*SCALE,  Scalar(0,   255,   0), 1, LINE_8); // Green
      circle(data, Point(cx, cy),     R_e*SCALE,  Scalar(255,   0,   0), 1, LINE_8); // Blue
    }

    imshow("LIDAR Datas", data);
  }
  return Ro_m;
}

int Lidar_Shutdown() {
  laser.turnOff();
  laser.disconnecting();
  return 0;
}
