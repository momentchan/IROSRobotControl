#pragma once
#include "main.h"

// Color Separation
void ColorSeparation(const Mat targetImg, vector<StrokeCluster> &fisrtDrawCluster);
bool CompareLength(vector<Point>, vector<Point>);

// Color Feedback
void colorDiffer(const Mat target, Mat detect, vector<pair <Point, float>> & drawPoints, float iteration);

// Strokes Generation
void EMapConstruct(Mat img, Mat & edgeMap, Mat & angles);
void SobelDetection(Mat src, Mat & grad, Mat & angles);
void CannyDetection(Mat src, Mat &canny);
void StrokesGeneration(const Mat img, Mat & canvas, const vector<pair <Point, float>> drawPoints, const Mat edgeMap, const Mat angles, float iteration);

// Camera Control
void DisplayInfo(const Mat image, Rect viewWindow, Stroke & stroke, char & color, float & level, Vec4f & CMYK);

// Robot Control
void setDefaultArmSpeed(float percentage = 1.0f);
vector<StrokeCluster> readFirstStroke(int & stroke_num, int &picture_id);
void MoveRelative(float x, float y, float z, float r, float p, float yaw);
void GoToPoint(float x, float y, float z, float theta_roll, float theta_pitch, float theta_yaw, float theta_arm);
void DipColor(float d);
void MixColor(int mix_times);
void DrawStroke(Stroke stroke);

// Utility
string outputFileName(string file_name, int index, string type);
float BilinearInterplation(float x, float y);
void ShowImg(string window_name, Mat img, int time = 0);
bool ColorDifferenceCompare(pair <Point, float> c1, pair <Point, float> c2);
void rgb2cmyk(const Vec3b bgr, Vec4f & cmyk);
string int2str(int &i);
vector<string> split(string, char);

