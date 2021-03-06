#include <iostream>
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <cstdio>
#include <string>
#include <sstream>
#include <fstream>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/objdetect/objdetect_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <limits>
#include <time.h> 
#include <conio.h>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace cv;

#define DRAWTHRESH 10// 20 for mean 50 for max 
#define CLUSTTHRESH 5
#define ITERATION 5
#define SAMPLEFREQ 50
#define PAUSE system("Pause");//fgetc(stdin);
#define MEANSHIFT 1
#define EDGECLIPING 1
#define TOLERANCE 15.0
#define PI 3.14159
#define DISPLAY 0
#define SIMULATION 0
#define CANNY 1
#define ROBOT_ON 1
#define pi_f 3.14159
#define MIN_DISTANCE 0.0
#define DIFFERTHRESH 50
#define MIXTIMES 2

// Data Structure
class Stroke{
public:
	Stroke(){}
	Stroke(Vec3b rgb, Vec4f cmyk, Point2f o, float r, float theta, float l);
	Stroke(Vec3b rgb, Vec4f cmyk, Point2f s, Point2f e, float r);
	void findEndpoint(const float, const float, const Mat edgeMap);
	float BilinearInterplation(float x, float y, const Mat edgeMap);
	void drawOnCanvas(Mat & canvas, const Mat edgeMap){
		findEndpoint(float(canvas.cols), float(canvas.rows), edgeMap);
		line(canvas, start, end, RGB, radius);
	}
	void drawOnCanvas(Mat & canvas){
		line(canvas, start, end, RGB, radius);
	}
	Vec3b getRGB(){ return RGB; }
	Vec4f getCMYK(){ return CMYK; }
	Point2f getPoint(int i){ 
		if (i == 0) return start; 
		else if (i == 1) return end;
	}
private:
	Vec3b RGB;
	Vec4f CMYK;
	Point2f center, start, end;
	float radius, dir_x, dir_y, length;
};

class StrokeCluster{
public:
	StrokeCluster(){ avgCMYK = Vec4f(0, 0, 0, 0); pointNum = 0;  maxInfo = make_pair(0, 0); }
	void addStroke(Stroke drawStroke);
	float computeDiffer(Vec4f cmyk);
	Stroke & getStroke(int index){ return drawStrokes[index]; }
	int getNum(){ return pointNum; }
	Vec4f getColor(){ return avgCMYK; }
	void showMaxInfo(){ cout << maxInfo.first << " " << maxInfo.second << endl; }
	pair<int, double> getMaxInfo(){ return maxInfo; }
	int getClusterID();
private:
	vector<Stroke> drawStrokes;
	Vec4f avgCMYK;
	int pointNum;
	pair<int, double> maxInfo;
};

