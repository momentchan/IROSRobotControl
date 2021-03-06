#include "FuncDeclaration.h"
extern Point mousePosition;
extern Mat TargetImg;
extern bool overlay;
extern Rect canvasView;
extern char DrawMode;
extern float mixThres, mixThresBlack;
int len = 5;
int writeOutID = 0;
extern bool showMarker;
void VisualFeedback(const Mat image, Rect viewWindow, Stroke &stroke, char & color, float & dip_level, Vec4f & CMYK){

	Vec3b targetRGB;
	Vec4f targetCMYK;
	// Mouse
	if (mousePosition.x != 0){
		targetRGB = image.at<Vec3b>(mousePosition.y, mousePosition.x);
		rgb2cmyk(targetRGB, targetCMYK);
		stroke = Stroke(targetRGB, targetCMYK, Point2f(0, 0), Point2f(0, 0), 2);
	}
	else{
		targetRGB = stroke.getRGB();
		targetCMYK = stroke.getCMYK();
	}
	// overlay
	if (overlay){
		Mat temp;
		Rect r;
		TargetImg.copyTo(temp);
		if (viewWindow.x + viewWindow.width < image.cols && viewWindow.y + viewWindow.height < image.rows){
			resize(temp, temp, Size(viewWindow.width, viewWindow.height));
			addWeighted(temp, 0.5, image(viewWindow), 0.5, 0.0, temp);
			temp.copyTo(image(viewWindow));
		}
		else if (viewWindow.x + viewWindow.width > image.cols && viewWindow.y + viewWindow.height < image.rows){
			r = Rect(viewWindow.x, viewWindow.y, image.cols - viewWindow.x, viewWindow.height);
			resize(temp, temp, Size(image.cols - viewWindow.x, viewWindow.height));
			addWeighted(temp, 0.5, image(r), 0.5, 0.0, temp);
			temp.copyTo(image(r));
		}
		else if (viewWindow.x + viewWindow.width < image.cols && viewWindow.y + viewWindow.height > image.rows){
			r = Rect(viewWindow.x, viewWindow.y, viewWindow.width, image.rows - viewWindow.y);
			resize(temp, temp, Size(viewWindow.width, image.rows - viewWindow.y));
			addWeighted(temp, 0.5, image(r), 0.5, 0.0, temp);
			temp.copyTo(image(r));
		}
		else{
			r = Rect(viewWindow.x, viewWindow.y, image.cols - viewWindow.x, image.rows - viewWindow.y);
			resize(temp, temp, Size(image.cols - viewWindow.x, image.rows - viewWindow.y));
			addWeighted(temp, 0.5, image(r), 0.5, 0.0, temp);
			temp.copyTo(image(r));
		}
		canvasView = viewWindow;
	}
	Point center = Point(viewWindow.x + viewWindow.width / 2, viewWindow.y + viewWindow.height / 2);

	
	// Decide initial Color
	int totalColorToDraw = 0;
	float totalMaxDiffer = INFINITE;
	Vec3b totalRGB;
	Vec4f totalCMYK;
	targetCMYK *= 100. / 255.;

	for (int i = center.x - len; i <= center.x + len; i += 2)
	for (int j = center.y - len; j <= center.y + len; j += 2){
			Vec3b rgb = image.at<Vec3b>(j, i);
			rgb2cmyk(rgb, CMYK);
	
			// Rescale to 100
			CMYK *= 100. / 255.;
			Vec4f differ = targetCMYK - CMYK;

			// Decide initial Color
			float maxDiffer = 0;
			int colorToDraw = 0;
			for (int i = 0; i < 4; i++){
				if (abs(int(differ[i]))>maxDiffer){
					if (int(differ[i]) > 0){
						colorToDraw = i;
					}
					else{
						colorToDraw = 4;
					}
					maxDiffer = abs(int(differ[i]));
				}
			}
			if (maxDiffer < totalMaxDiffer){
				totalMaxDiffer = maxDiffer;
				totalRGB = rgb;
				totalCMYK = CMYK;
				totalColorToDraw = colorToDraw;
			}
		}
	// Rescale to 100
	CMYK = totalCMYK;
	Vec4f differ = targetCMYK - CMYK;

	

	//system("cls");
	char name[30];
	int w = image.cols, h = image.rows;
	int patch_size = 50;


	// Draw marker
	if (showMarker){
	line(image, Point(center.x - 10, center.y), Point(center.x + 10, center.y), Scalar(0, 0, 255));
	line(image, Point(center.x, center.y - 10), Point(center.x, center.y + 10), Scalar(0, 0, 255));
	}
	
	// Color Information
	sprintf_s(name, "C=%d", (int)CMYK[0]);
	putText(image, name, Point(25, 40), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "M=%d", (int)CMYK[1]);
	putText(image, name, Point(25, 80), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "Y=%d", (int)CMYK[2]);
	putText(image, name, Point(25, 120), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "K=%d", (int)CMYK[3]);
	putText(image, name, Point(25, 160), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	// Position Information
	sprintf_s(name, "X=%d", viewWindow.x);
	putText(image, name, Point(25, 200), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "Y=%d", viewWindow.y);
	putText(image, name, Point(25, 240), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "W=%d", viewWindow.width);
	putText(image, name, Point(25, 280), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "H=%d", viewWindow.height);
	putText(image, name, Point(25, 320), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	rectangle(image, Point(w - 2 * patch_size, 0), Point(w - patch_size, patch_size), totalRGB, -1, 8);
	rectangle(image, Point(w - patch_size, 0), Point(w, patch_size), targetRGB, -1, 8);
	if (showMarker){
		rectangle(image, Point(viewWindow.x, viewWindow.y), Point(viewWindow.x + viewWindow.width, viewWindow.y + viewWindow.height), (255, 0, 0), 2);
	}
	

	printf("\n\n Color Detection\n");
	printf(" Target: (%d,%d,%d,%d)\n", (int)targetCMYK[0], (int)targetCMYK[1], (int)targetCMYK[2], (int)targetCMYK[3]);
	printf(" Detect: (%d,%d,%d,%d)\n", (int)CMYK[0], (int)CMYK[1], (int)CMYK[2], (int)CMYK[3]);
	printf(" Differ: (%d,%d,%d,%d)\n\n", (int)differ[0], (int)differ[1], (int)differ[2], (int)differ[3]);

	
	dip_level = round(totalMaxDiffer);
	if (DrawMode == 'm'){
		if (targetCMYK[3] > 60){
			if (totalMaxDiffer < mixThresBlack)
				totalColorToDraw = 5;
		}
		else{
			if (totalMaxDiffer < mixThres)
				totalColorToDraw = 5;
		}
	}

	switch (totalColorToDraw){
	case 0:
		printf(" Draw: Cyan %d", (int)dip_level);
		color = 'C';
		break;
	case 1:
		printf(" Draw: Magenta %d", (int)dip_level);
		color = 'M';
		break;
	case 2:
		printf(" Draw: Yellow %d", (int)dip_level);
		color = 'Y';
		break;
	case 3:
		printf(" Draw: Black %d", (int)dip_level);
		color = 'K';
		break;
	case 4:
		printf(" Draw: White %d", (int)dip_level);
		color = 'W';
		break;
	case 5:{
		writeOutID++;
		Mat patch = Mat(Size(patch_size, patch_size), CV_8UC3);
		patch.setTo(targetRGB);
		string name = outputFileName("patch/target", writeOutID, ".jpg");
		imwrite(name, patch);

		patch.setTo(totalRGB);
		name = outputFileName("patch/detect", writeOutID, ".jpg");
		imwrite(name, patch);
		
		printf(" Color mixing done!");
		color = 'N';
		break;
		}
	}
	imshow("Capture", image);
	cvWaitKey(33);
}