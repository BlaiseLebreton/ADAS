#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>

#define LIAISON 0
#ifdef LIAISON
  #include "liaison.h"
#endif

using namespace cv;
using namespace std;

void LineAlignement(int event, int x, int y, int flags, void* userdata);
void RegionOfInterest(int event, int x, int y, int flags, void* userdata);
void polyfit(const Mat& src_x, const Mat& src_y, Mat& dst, int order);

#define THICK 1
#define DEBUG 2

// Warping
int i = 0; // indice du point de warping
int warp_factor = 1; // facteur d'agrandissement en y
vector<Point2f> pts_src; // point sur la frame raw pour warping
vector<Point2f> pts_dst; // point transformes sur la frame warp
Mat h, hinv; // matrice de passage raw -> warp et warp -> raw

// Region of interest
Rect myROI(74, 144, 171, 87); // region traitee
Point point1, point2; // utilises pour la definition de cette region
int drag = 0;

// Sobel
int scale = 2; // facteur pour "boost" l'intensite
int delta = 0;
int ddepth = CV_16S;
int threshold_sobel = 30; // filtre

// Sliding windows
int n_win = 10; // nombre de fenetres
int win_width = 30; // largeur en px d'une fenetre
int min_points = 0; // nombre minimum de points dans la fenetre
int degre = 2; // degre du polynome pour relier x = f(y)
struct Window{
  // Coordonnes de la ligne
  Point2f lane;
  // Region de recherche
  Rect rect;
  // Il y a t il assez de points dans la region ?
  bool detected;
};

// PID
int posy;
int posx;
double curr_error = 0.0;
double prev_error = 0.0;
double kp = 1.0;
double ki = 0.0;
double kd = 0.0;
int dir = 0;
int pwr = 0;

// images pour traitement
Mat raw, warp, crop, hsv, sobel, slid, warpback;
vector<Mat> hsv_chan(3);

int main(int argc, char** argv) {
  setUseOptimized(1);

  if (LIAISON == 1) {
    if (Liaison_Initialize() == 1) {
      return 1;
    }
    else {
      printf("Comport opened\n");
    }
  }

  // Derivee de l'intensite selon x
	Mat grad_x;

	// Filtre de sobel
	double vmin, vmax;

	// Sliding windows
	Mat roi;
	int xc, sumxi, sumi, n;

	// Loops
	int c, nw, r, s;

	// Temps d'execution
	double start,stop, dt;

	// Initialisation de la camera et lecture d'une frame
  VideoCapture cap;
	if (argc == 1) {
    int apiID = CAP_ANY;
    for (int deviceID = 9; deviceID >= 0; deviceID--) {
      cap.open(deviceID + apiID);
      if (cap.isOpened()) {
        break;
      }
    }

		//Definition de la resolution
		cap.set(CAP_PROP_FRAME_WIDTH,  640);
		cap.set(CAP_PROP_FRAME_HEIGHT, 360);
		cap >> raw;
	}
	else{
		raw = imread(argv[1], IMREAD_COLOR);
	}

	// Calculate Homography
	pts_src.push_back(Point2f(3, 129));
	pts_src.push_back(Point2f(102, 79));
	pts_src.push_back(Point2f(315, 130));
	pts_src.push_back(Point2f(221, 80));

  // Perspective transformee : Lignes deviennent verticales
	pts_dst.push_back(Point2f(pts_src.at(1).x, 0));
	pts_dst.push_back(Point2f(pts_src.at(1).x, 0));
	pts_dst.push_back(Point2f(pts_src.at(3).x, 0));
	pts_dst.push_back(Point2f(pts_src.at(3).x, 0));

	// Application du facteur sur y pour augmenter la hauteur du warp
	pts_dst.at(0).y = warp_factor*raw.rows;
	pts_dst.at(2).y = warp_factor*raw.rows;
	pts_dst.at(1).y = warp_factor*raw.rows*1/2;
	pts_dst.at(3).y = warp_factor*raw.rows*1/2;

	// Calcul des matrices de passage
	h    = findHomography(pts_src, pts_dst);
	hinv = findHomography(pts_dst, pts_src);

  // Creation des fenetres
	namedWindow("Raw",            WINDOW_NORMAL);
	namedWindow("Warp",           WINDOW_NORMAL);
	namedWindow("Sliding window", WINDOW_NORMAL);
	//namedWindow("PID",            WINDOW_NORMAL);

  // Creation callback / trackbar
  createTrackbar("Seuil",   "Sliding window", &threshold_sobel,      255);
  createTrackbar("Nombre",  "Sliding window", &n_win,                 50);
  createTrackbar("Largeur", "Sliding window", &win_width,            100);
  createTrackbar("MinPix",  "Sliding window", &min_points,          1000);
  createTrackbar("Posy",    "Warp",           &posy,            raw.rows);
  createTrackbar("Posx",    "Warp",           &posx,            raw.cols);
  //createTrackbar("Kp",      "PID",            &kp,                   100);
  //createTrackbar("Ki",      "PID",            &ki,                   100);
  //createTrackbar("Kd",      "PID",            &kd,                   100);

  // Callback
  setMouseCallback("Warp", RegionOfInterest, NULL);
  setMouseCallback("Raw",  LineAlignement,   NULL);

  // Redimensionnement
	resizeWindow("Raw",            640,               360);
	resizeWindow("Warp",           640, warp_factor * 360);
	resizeWindow("Sliding window", 640, warp_factor * 360);

  // Definition des points de bases du PID
	posx = raw.cols/2;
	posy = raw.rows/4;

	// Debut du traitement temps reel
	cout << "Debut de capture" << endl;
	for(;;)	{


    /* RECUPERATION D'UNE FRAME */

    // Lecture frame
		cap >> raw;
		if (argc == 1)
			cap >> raw;
		else
			raw = imread(argv[1], IMREAD_COLOR);

		// Verification frame non vide
		if (raw.empty()) {
			cerr << "Image vide\n";
			return -1;
		}

    // Calcul du temps d'execution
    start = getTickCount();

		// Rotation 180°
    flip(raw, raw, 0);


		// Transformation en bird view
    warpPerspective(raw, warp, h, Size(raw.cols, warp_factor * raw.rows));
		line(raw,  Point(pts_src.at(0)), Point(pts_src.at(1)), Scalar(255,0,0));
		line(raw,  Point(pts_src.at(2)), Point(pts_src.at(3)), Scalar(255,0,0));
		// line(warp, Point(pts_dst.at(0)), Point(pts_dst.at(1)), Scalar(255,0,0));
		// line(warp, Point(pts_dst.at(2)), Point(pts_dst.at(3)), Scalar(255,0,0));

		// Crop de la frame
		crop = warp(myROI);


    /* SOBEL */

    // Reduction bruit
		GaussianBlur(crop, crop, Size(3, 3), 0, 0, BORDER_DEFAULT);

		// Conversion en HSV
    cvtColor(crop, hsv, COLOR_BGR2HSV);

    // Separation canaux
    split(hsv, hsv_chan);

		// Filtre de Sobel
		Sobel(hsv_chan[0], grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
		convertScaleAbs(grad_x, sobel);

		// Mise a l'echelle de la frame
		minMaxLoc(sobel, &vmin, &vmax);
		sobel = 255 * sobel / vmax;

		// Seuillage a zero
		threshold(sobel, sobel, threshold_sobel, 255, 3);


    /* ALGORITHME DES FENETRES GLISSANTES */

    //conversion en couleur pour representation des fenetres
		cvtColor(sobel, slid, COLOR_GRAY2BGR);

    // A utiliser plutot que gauche/droite et parametrer le 2
    vector <vector<Window>> slid_win;
    slid_win.resize(2, vector<Window>(n_win));

    // Definition de la hauteur et largeur des fenetres
    for (nw = 0; nw < n_win; nw++) {
      for (s = 0; s < 2; s++) {
        if (nw == 0)
          slid_win[s][nw].rect.height = sobel.rows - (n_win-1)*floor(sobel.rows/n_win);
        else
          slid_win[s][nw].rect.height = floor(sobel.rows/n_win);
        slid_win[s][nw].rect.width  = win_width;
      }
    }

		// Placement de la premiere fenetre
    for (s = 0; s < 2; s++) {
      slid_win[s][0].rect.width = sobel.cols / 2;
      slid_win[s][0].rect.y     = sobel.rows - slid_win[0][0].rect.height;
    }
		slid_win[0][0].rect.x = 0;
		slid_win[1][0].rect.x = sobel.cols / 2;

		// Parcours de toute les fenetres
    for (nw = 0; nw < n_win; nw++) {
      for (s = 0; s < 2; s++) {

        // Selection de la sous-region
        roi = sobel(slid_win[s][nw].rect);
        sumxi = 0; sumi = 0; n = 0;

        // Calcul du x moyen pondere
        for (int r = 0; r < roi.rows; r++) {
          for (int c = 0; c < roi.cols; c++) {
            if (roi.at<uchar>(r, c) > 0) {
              sumxi += roi.at<uchar>(r, c) * c;
              sumi  += roi.at<uchar>(r, c);
              n++;
            }
          }
        }

        // Si il y a assez de points, on place la fenetre sur le x moyen
        if (n > min_points) {
          slid_win[s][nw].detected = true;
          slid_win[s][nw].lane.x = myROI.x + slid_win[s][nw].rect.x + sumxi / sumi;
          slid_win[s][nw].lane.y = myROI.y + slid_win[s][nw].rect.y + slid_win[s][nw].rect.height / 2;
          if (nw + 1 < n_win){
            xc = slid_win[s][nw].rect.x + sumxi / sumi - win_width / 2;
            if (s == 0)
              xc = (xc > 0) ? xc : 0;
            else
              xc = (xc + win_width < sobel.cols) ? xc : sobel.cols - win_width;
            slid_win[s][nw + 1].rect.x = xc;
          }
        }
        // Sinon on augmente la zone de recherche pour la fenetre suivante
        else {
          slid_win[s][nw].detected = false;

          if (nw - 1 > 0){
            slid_win[s][nw].lane.x = slid_win[s][nw-1].lane.x;
            slid_win[s][nw].lane.y = slid_win[s][nw-1].lane.y - slid_win[1][nw-1].rect.height;
          }
          if (nw + 1 < n_win){
            if (s == 0)
              slid_win[s][nw + 1].rect.x = 0;
            else
              slid_win[s][nw + 1].rect.x = slid.cols / 2;
            slid_win[s][nw + 1].rect.width = slid.cols / 2;
          }
        }

        // Calcul de la prochaine position
        if (nw + 1 < n_win){
          slid_win[s][nw + 1].rect.y = slid_win[s][nw].rect.y - slid_win[s][nw + 1].rect.height;
        }
      }
    }

    // Affichage puis desactivation de la premiere fenetre (car position erronee)
    for (s = 0; s < 2; s++) {
      if (slid_win[s][0].detected)
        rectangle(slid, slid_win[s][0].rect, Scalar(255, 0, 0), THICK);
      else
        rectangle(slid, slid_win[s][0].rect, Scalar(0, 0, 255), THICK);
      slid_win[s][0].detected = false;
    }

		// Affichage des fenetres et calcul des coordonnes des lignes
		vector<Rect> center(n_win);
    int nc = 0;
    for (nw = 1; nw < n_win; nw++) {
      for (s = 0; s < 2; s++) {

        if(slid_win[s][nw].detected){
          rectangle(slid, slid_win[s][nw].rect, Scalar(0, 255, 0), THICK);
          circle(warp, slid_win[s][nw].lane, THICK, Scalar(255, 255, 255), FILLED, 8, 0);
        }
        else{
          rectangle(slid, slid_win[s][nw].rect, Scalar(0, 0, 255), THICK);
        }
      }

      if(slid_win[0][nw].detected && slid_win[1][nw].detected){
        center[nc].x = (float)(slid_win[1][nw].lane.x + slid_win[0][nw].lane.x) / 2;
        center[nc].y = (float)(slid_win[1][nw].lane.y + slid_win[0][nw].lane.y) / 2;
        // circle(warp, Point(center[nc].x, center[nc].y), 2, Scalar(0, 0, 255), FILLED, 8, 0);
        nc++;
      }
    }

    /* CALCUL DE L'EQUATION DE LA ROUTE x = f(y) */

    // Matrice des coefs
    Mat coef = Mat(degre + 1, 1, CV_32F);
    if (nc > 0){

      // Mise en forme des i/o
  		Mat src_x = Mat(nc, 1, CV_32F);
  		Mat src_y = Mat(nc, 1, CV_32F);
      for(int n = 0; n < nc; n++){
    		src_x.at<float>(n, 0) = center[n].y;
    		src_y.at<float>(n, 0) = center[n].x;
      }
  		polyfit(src_x, src_y, coef, degre);

      // Affichage de la ligne resultante
      vector<Point2f> curvePoints;
      int x = 0;
  		for (int y = warp_factor * warp.rows; y > 0; y--) {
        x = 0;
  		  for (int n = 0; n <= degre; n++) {
  		 		x += coef.at<float>(n, 0) * pow(y, n);
  		 	}
        curvePoints.push_back(Point2f(x, y));
  		}
      Mat curve(curvePoints, true);
      curve.convertTo(curve, CV_32S);
      polylines(warp, curve, false, Scalar(255, 255, 255), 1, LINE_AA);
    }


    /* PID */

    // Calcul du temps
    stop = getTickCount();
    dt = ((stop - start)/ getTickFrequency());

    // Affichage de la ligne
    int yligne = warp.rows - posy;
    line(warp, Point(posx, warp.rows), Point(posx, yligne), Scalar(255,0,0), THICK);

    // Calcul de l'erreur
    int xligne = 0;
	  for (int n = 0; n <= degre; n++) {
	 		xligne += coef.at<float>(n, 0) * pow(yligne, n);
	 	}
    line(warp, Point(posx, warp.rows - posy), Point(xligne, yligne), Scalar(0,0,255), THICK);
    curr_error = xligne - posx;

    // Calcul de la commande
		dir = kp*curr_error + ki*curr_error*dt + kd*(curr_error - prev_error)/dt;
    putText(warp, to_string(dir), Point(warp.cols/2,yligne-5*THICK), FONT_HERSHEY_DUPLEX, 0.5*THICK, Scalar(255,255,255), 2);
		pwr = 1.0/coef.at<float>(2, 0);
		if (pwr < 0)
			pwr = -pwr;
		putText(warp, to_string(pwr), Point(warp.cols/2,yligne-25*THICK), FONT_HERSHEY_DUPLEX, 0.5*THICK, Scalar(255,255,255), 2);
    prev_error = curr_error;

    // Bornage
    dir = min(max(-90,  dir),   90);
    pwr = min(max(1300, pwr), 1800);

    // Override
    pwr = 1500;

    // Envoi de la commande a l'arduino
    if (LIAISON == 1) {
      Liaison_SendData(dir,pwr);
    }

    /* AFFICHAGE */

    // Ligne d'erreur
		line(raw, Point(0, pts_src.at(0).y), Point(raw.cols, pts_src.at(0).y), Scalar(255,0,0), THICK);
		line(raw, Point(0, pts_src.at(1).y), Point(raw.cols, pts_src.at(1).y), Scalar(255,0,0), THICK);

    // Frequence de traitement
    stop = getTickCount();
    dt = ((stop - start)/ getTickFrequency());
    putText(raw, to_string((int)(1/dt)) + "Hz", Point(10,15), FONT_HERSHEY_DUPLEX, 0.5*THICK, Scalar(255,255,255), 2);

		// Frames et creation callback/trackbar
		imshow("Raw",             raw);
		imshow("Warp",           warp);
		imshow("Sliding window", slid);
		//imshow("PID",               0);

		if (waitKey(1) == 27) {
      break; // stop capturing by pressing ESC
    }

	}

	cout << "Fin de capture" << endl;
	return 0;
}

// Callback pour calcul des points de transformation
void LineAlignement(int event, int x, int y, int flags, void* userdata) {
	if (event == EVENT_LBUTTONDOWN)	{
		if (i >= 0 && i <= 3) {
      if (DEBUG > 0)
			   cout << "src[" << i << "] : " << x << ", " << y << endl;
			(pts_src.at(i)).x = x;
			(pts_src.at(i)).y = y;
		}
		if (i == 3) {
			pts_dst.at(0).x = pts_src.at(1).x;
			pts_dst.at(1).x = pts_src.at(1).x;
			pts_dst.at(2).x = pts_src.at(3).x;
			pts_dst.at(3).x = pts_src.at(3).x;

			pts_dst.at(0).y = warp_factor*raw.rows;
			pts_dst.at(2).y = warp_factor*raw.rows;
			pts_dst.at(1).y = warp_factor*raw.rows*1/2;
			pts_dst.at(3).y = warp_factor*raw.rows*1/2;

			// for (int j = 0; j < 4; j++) {
			// 	pts_dst.at(j).y = (warp_factor - 1) * raw.rows + pts_src.at(j).y;
			// }

			h    = findHomography(pts_src, pts_dst);
			hinv = findHomography(pts_dst, pts_src);
		}
		i++;
		if (i > 3) {
			i = 0;
		}
	}
}

// Callback pour calcul de la region traitee
void RegionOfInterest(int event, int x, int y, int flags, void* userdata) {
  // LMB clicked. ROI selection begins
	if (event == EVENT_LBUTTONDOWN && !drag) {
		point1 = Point(x, y);
		drag = 1;
	}

  // LMB released. ROI end selection
	if (event == EVENT_LBUTTONUP && drag) {
    point2 = Point(x, y);
		if (x - point1.x > 0 && y - point1.y > 0)
			myROI = Rect(point1.x, point1.y, x - point1.x, y - point1.y);
		else
			myROI = Rect(point1.x, point1.y, 1, 1);

    if (DEBUG > 0)
		  cout << "myROI(" << myROI.x << ", " << myROI.y << ", " << myROI.width << ", " << myROI.height << ")" << endl;
		drag = 0;
	}

	if (event == EVENT_LBUTTONUP) {
		/* ROI selected */
		drag = 0;
	}
}

// Fonction pour determiner les coefficients du polynome de x = f(y)
void polyfit(const Mat& src_x, const Mat& src_y, Mat& dst, int order) {
	CV_Assert((src_x.rows > 0) && (src_y.rows > 0) && (src_x.cols == 1) && (src_y.cols == 1) && (dst.cols == 1) && (dst.rows == (order + 1)) && (order >= 1));
	Mat X;
	X = Mat::zeros(src_x.rows, order + 1, CV_32FC1);
	Mat copy;

	for (int i = 0; i <= order; i++) {
		copy = src_x.clone();
		pow(copy, i, copy);
		Mat M1 = X.col(i);
		copy.col(0).copyTo(M1);
	}

	Mat X_t, X_inv;
	transpose(X, X_t);
	Mat temp = X_t * X;
	Mat temp2;
	invert(temp, temp2);
	Mat temp3 = temp2 * X_t;
	Mat W = temp3 * src_y;
	W.copyTo(dst);
}
