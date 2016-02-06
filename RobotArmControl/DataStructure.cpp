#include "FuncDeclaration.h"

// ************************
//  Stroke Member function
// ************************
Stroke::Stroke(Vec3b rgb, Vec4f cmyk, Point2f o, float r, float theta, float l){
	RGB = rgb;
	CMYK = cmyk;
	center = start = end = o;
	radius = r;
	dir_x = cos(theta);
	dir_y = sin(theta);
	float ratio = 0.8 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (1.2 - 0.8)));
	length = l*ratio;
};
Stroke::Stroke(Vec3b rgb, Vec4f cmyk, Point2f s, Point2f e, float r){
	RGB = rgb;
	CMYK = cmyk;
	start = s;
	end = e;
	radius = r;
}
void Stroke::findEndpoint(const float bound_x, const float bound_y, const Mat edgeMap){

	// Find Start Point
	float start_x = center.x, start_y = center.y;
	float lastSample = edgeMap.at<uchar>(start_y, start_x);
	while (1){
		float temp_x = start_x + dir_x;
		float temp_y = start_y + dir_y;
		float distance = sqrtf(pow(temp_x - center.x, 2) + pow(temp_y - center.y, 2));
		if (temp_x < 0 || temp_x > bound_x - 1 || temp_y < 0 || temp_y > bound_y - 1 || distance > length / 2)
			break;
		else{
#if EDGECLIPING
#if CANNY
			float newSample = BilinearInterplation(temp_x, temp_y, edgeMap);
			if (newSample > 10.0)
				break;
#else
			float newSample = BilinearInterplation(temp_x, temp_y, edgeMap);
			if (newSample > lastSample)
				break;
			lastSample = newSample;
#endif
#endif
			start_x = temp_x;
			start_y = temp_y;
		}
	}
	start = Point2f(start_x, start_y);

	// Find End Point
	float end_x = center.x, end_y = center.y;
	lastSample = edgeMap.at<uchar>(end_y, end_x);
	while (1){
		float temp_x = end_x - dir_x;
		float temp_y = end_y - dir_y;
		float distance = sqrtf(pow(temp_x - center.x, 2) + pow(temp_y - center.y, 2));
		if (temp_x < 0 || temp_x > bound_x - 1 || temp_y < 0 || temp_y > bound_y - 1 || distance > length / 2)
			break;
		else{
#if EDGECLIPING
#if CANNY
			float newSample = BilinearInterplation(temp_x, temp_y, edgeMap);
			if (newSample > 10.0)
				break;
#else
			float newSample = BilinearInterplation(temp_x, temp_y, edgeMap);
			if (newSample > lastSample)
				break;
			lastSample = newSample;
#endif
#endif
			end_x = temp_x;
			end_y = temp_y;
		}
	}
	end = Point2f(end_x, end_y);
}
float Stroke::BilinearInterplation(float x, float y, const Mat edgeMap){
	float ceil_x = ceil(x);
	float floor_y = floor(y);
	float floor_x = floor(x);
	float ceil_y = ceil(y);

	float leftTop = edgeMap.at<uchar>(floor_y, floor_x);
	float rightTop = edgeMap.at<uchar>(floor_y, ceil_x);
	float leftBottom = edgeMap.at<uchar>(ceil_y, floor_x);
	float rightBottom = edgeMap.at<uchar>(ceil_y, ceil_x);
	return ((x - floor_x)*rightTop + (ceil_x - x)*leftTop)*(ceil_y - y) + ((x - floor_x)*rightBottom + (ceil_x - x)*leftBottom)*(y - floor_y);
}

// *******************************
//  StrokeCluster Member function
// *******************************
void StrokeCluster::addStroke(Stroke drawStroke){
	drawStrokes.push_back(drawStroke);
	pointNum += 1;

	Vec3b color = drawStroke.getRGB();
	Vec4f CMYK;
	rgb2cmyk(color, CMYK);
	avgCMYK = (avgCMYK * float(pointNum - 1) + CMYK) / float(pointNum);
	minMaxIdx(avgCMYK, NULL, &maxInfo.second, NULL, &maxInfo.first);
}
float StrokeCluster::computeDiffer(Vec4f cmyk){
	Vec4f differ;
	absdiff(cmyk, avgCMYK, differ);
	float differMean = mean(differ)[0];
	return differMean;
}