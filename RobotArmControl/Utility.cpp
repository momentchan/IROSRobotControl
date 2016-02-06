#include "FuncDeclaration.h"

string int2str(int &i) {
	string s;
	stringstream ss(s);
	ss << i;
	return ss.str();
}
string outputFileName(string file_name, int index, string type){
	file_name.append(int2str(index));
	file_name.append(type);
	return file_name;
}
void rgb2cmyk(const Vec3b bgr, Vec4f & cmyk) {
	int B = bgr.val[0];
	int G = bgr.val[1];
	int R = bgr.val[2];
	float r = R / 255.;
	float g = G / 255.;
	float b = B / 255.;
	float k = std::min(std::min(1 - r, 1 - g), 1 - b);

	cmyk[0] = (1 - r - k) / (1 - k) * 255.;
	cmyk[1] = (1 - g - k) / (1 - k) * 255.;
	cmyk[2] = (1 - b - k) / (1 - k) * 255.;
	cmyk[3] = k * 255.;
}
void ShowImg(string window_name, Mat img){
	imshow(window_name, img);
	waitKey(0);
}
bool ColorDifferenceCompare(pair <Point, float> c1, pair <Point, float> c2) {
	float i = c1.second;
	float j = c2.second;
	return (i > j);
}
bool CompareLength(vector<Point> contour1, vector<Point> contour2) {
	double i = arcLength(contour1, false);
	double j = arcLength(contour2, false);
	return (i > j);
}
vector<string> split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str); // Turn the string into a stream.
	string tok;
	while (getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}
	return internal;
}
