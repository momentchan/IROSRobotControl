#include "FuncDeclaration.h"

void SobelDetection(Mat src, Mat & grad, Mat & angles){
	int scale = 1, delta = 0;
	int ddepth = CV_16S;
	int c;
	Mat src_gray;
	GaussianBlur(src, src, Size(3, 3), 0, 0);
	cvtColor(src, src_gray, CV_BGR2GRAY);

	/// Generate grad_x and grad_y
	Mat abs_grad_x, abs_grad_y;
	Mat grad_x, grad_y;
	/// Gradient X
	Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta);
	convertScaleAbs(grad_x, abs_grad_x);
	/// Gradient Y
	Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta);
	convertScaleAbs(grad_y, abs_grad_y);
	/// Total Gradient (approximate)
	//addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
	for (int y = 0; y < grad.rows; y++)
	for (int x = 0; x < grad.cols; x++)
		grad.at<uchar>(y, x) = sqrt(pow(abs_grad_x.at<uchar>(y, x), 2) + pow(abs_grad_y.at<uchar>(y, x), 2));

	//to store the gradients grad_x.convertTo(grad_x,CV_32F);
	grad_x.convertTo(grad_x, CV_32FC1);
	grad_y.convertTo(grad_y, CV_32FC1);

	phase(grad_x, grad_y, angles, false);
}
void CannyDetection(Mat src, Mat &canny){
	Mat gray;
	blur(src, gray, Size(3, 3));
	Canny(gray, canny, 100, 150, 3);
}

void EMapConstruct(Mat img, Mat & edgeMap, Mat & angles){
	// Get edgeMap by Sobel
	edgeMap = Mat(img.rows, img.cols, CV_8UC1);
	angles = Mat::zeros(img.rows, img.cols, CV_32F);;
	SobelDetection(img, edgeMap, angles);
#if CANNY
	CannyDetection(img, edgeMap);
#endif

#if DISPLAY
	ShowImg("Orientation", angles, 0);
	ShowImg("EdgeMap", edgeMap, 0);
#endif
	Mat orientation;
	normalize(angles, orientation, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
	imwrite("Image/Orientation.jpg", orientation);
	imwrite("Image/EdgeMap.jpg", edgeMap);
}

bool ColorSort(StrokeCluster c1, StrokeCluster c2){
	int t1 = c1.getClusterID();
	int t2 = c2.getClusterID();
	if (t1 < t2)
		return true;
	else if (t1 > t2)
		return false;

 	pair <int, double> p1 = c1.getMaxInfo();
	pair <int, double> p2 = c2.getMaxInfo();
	if (t1 != 0){
		/*if (p1.second > p2.second)
			return true;
		return false;*/
		if (c1.getColor()[3] < c2.getColor()[3])
			return true;
		return false;
	}
	else{
		if (c1.getColor()[3] < c2.getColor()[3])
			return true;
		return false;
	}
	return false;
}
vector<StrokeCluster> StrokesGeneration(const Mat img, Mat & canvas, const vector<pair <Point, float>> drawPoints, const Mat edgeMap, const Mat angles, float iteration){
	
	vector<StrokeCluster> StrokeClusters;
	
	// Initial Stroke Cluster
	StrokeClusters.push_back(StrokeCluster());
	int x = drawPoints[0].first.x;
	int y = drawPoints[0].first.y;
	Vec3b rgb = img.at<Vec3b>(y, x);
	Vec4f cmyk;
	rgb2cmyk(rgb, cmyk);
	float angle = angles.at<float>(y, x) + PI / 2;
	StrokeClusters[0].addStroke(Stroke(rgb, cmyk, Point2f(x, y), 10.0 / iteration, angle, 20.0));


	// Color clustering
	for (int i = 1; i < drawPoints.size(); i++){
		float minDiffer = INFINITE;
		Stroke stroke;
		int bestIndex = 0;
		for (int c = 0; c < StrokeClusters.size(); c++){
			int x = drawPoints[i].first.x;
			int y = drawPoints[i].first.y;
			Vec3b rgb = img.at<Vec3b>(y, x);
			Vec4f cmyk;
			rgb2cmyk(rgb, cmyk);
			float colorDiffer = StrokeClusters[c].computeDiffer(cmyk);
			if (colorDiffer < minDiffer){
				bestIndex = c;
				minDiffer = colorDiffer;
				float angle = angles.at<float>(y, x) + PI / 2;
				stroke = Stroke(rgb, cmyk, Point2f(x, y), 10.0 / iteration, angle, 20.0);
			}
		}
		// Add into cluster
		if (minDiffer < CLUSTTHRESH){
			StrokeClusters[bestIndex].addStroke(stroke);
		}
		// Construct a new cluster
		else{
			StrokeClusters.push_back(StrokeCluster());
			StrokeClusters[StrokeClusters.size() - 1].addStroke(stroke);
		}
	}
	// Color Sorting
	sort(StrokeClusters.begin(), StrokeClusters.end(), ColorSort);
	/*for (int c = 0; c < StrokeClusters.size(); c++){
		StrokeClusters[c].showMaxInfo();
	}*/
	return StrokeClusters;


}