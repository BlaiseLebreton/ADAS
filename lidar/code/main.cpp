/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
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
using namespace std;
using namespace ydlidar;
using namespace cv;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

int main(int argc, char *argv[]) {
  std::string port;
  ydlidar::init(argc, argv);

  std::map<std::string, std::string> ports = ydlidar::YDlidarDriver::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
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

  int baudrate = 115200;
  bool isSingleChannel = true;
  bool isTOFLidar = true;
  float frequency = 8.0;

  if (!ydlidar::ok()) {
    return 0;
  }

  CYdLidar laser;
  laser.setSerialPort(port);
  laser.setSerialBaudrate(baudrate);
  laser.setFixedResolution(false);
  laser.setReversion(false);
  laser.setInverted(false);
  laser.setAutoReconnect(true);
  laser.setSingleChannel(isSingleChannel);
  laser.setLidarType(isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);
  laser.setMinRange(0.01);
  laser.setMaxRange(8.0);
  laser.setScanFrequency(frequency);
  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  int WIDTH = 800;
  int SCALE = WIDTH/(8.0*2.0);
  Mat data = Mat::zeros(Size(WIDTH, WIDTH), CV_8UC3);

  // Create window
  namedWindow("Raw", WINDOW_NORMAL);
  resizeWindow("Raw", WIDTH, WIDTH);

  Mat plot_result;
  int sx,sy,cx,cy,psi = 0;
  float R,R_g,R_d,E=0.3,L=0.2;

  createTrackbar("Angle braquage", "Raw", &psi,  90);
  setTrackbarMin("Angle braquage", "Raw",       -90);

  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;
    data = 0;

    R = E/cos((90-psi)*2*3.14/360.0); // calcul de rayon de la trajectoire en fonction de l'empattement et de rayon de braquage

    R_g = R - (L/2);
    R_d = R + (L/2);

    cy = WIDTH/2;
    cx = WIDTH/2 - R*SCALE;
    circle(data, Point(cx, cy), abs(R   *SCALE), Scalar(0,     0, 255), 1, LINE_8);
    circle(data, Point(cx, cy), abs(R_g *SCALE), Scalar(0,   255,   0), 1, LINE_8);
    circle(data, Point(cx, cy), abs(R_d *SCALE), Scalar(255,   0,   0), 1, LINE_8);

    if (laser.doProcessSimple(scan, hardError)) {
      for(int i = 0; i < scan.points.size(); i++) {
        sx =  round(scan.points[i].range*SCALE*cos(scan.points[i].angle)) + WIDTH/2;
        sy = -round(scan.points[i].range*SCALE*sin(scan.points[i].angle)) + WIDTH/2;

        // if (Coord_x <=R_d & Coord_x >= R_g & Coord_y <=R_d & Coord_y >= R_g) {
        //   data.at<Vec3b>(Coord_x, Coord_y) = Vec3b(255,0,0);
        //   printf("Attention!!!");
        // }
        if (sx <= WIDTH/2) {
          data.at<Vec3b>(sx, sy) = Vec3b(0,0,255);
        }
        else {
          data.at<Vec3b>(sx, sy) = Vec3b(0,255,0);
        }
      }
    }
    else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }

    imshow("Raw", data);
    if (waitKey(1) == 27)
      break; // stop capturing by pressing ESC

  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
