#include "FuncDeclaration.h"
#include "LuoLitaArm.h"
extern Vector6f t;
extern Matrix4f T;
extern VideoCapture captureDevice;
extern Stroke stroke;
extern Vec4f CMYK;
extern Mat detectImg;
extern Point2f canvas_center;
extern char mix_color;
extern float level;
extern float paperSize;
extern float imageSize;
extern float board_touch;
extern float view_dz;

void idleDisplay(){
	while (MOVL){
		if (_kbhit())
			kbCmd = _getche();
		system("cls");
		DisplayLoop();
		if (mix_color == 'C')
			printf(" Draw: Cyan %d", (int)level);
		else if (mix_color == 'M')
			printf(" Draw: Magenta %d", (int)level);
		else if (mix_color == 'Y')
			printf(" Draw: Yellow %d", (int)level);
		else if (mix_color == 'K')
			printf(" Draw: White %d", (int)level);

		printf("\n\n Color Detection\n");
		printf(" Target: (%d,%d,%d,%d)\n", (int)(stroke.getCMYK()[0] * 100. / 255.), (int)(stroke.getCMYK()[1] * 100. / 255.),
			(int)(stroke.getCMYK()[2] * 100. / 255.), (int)(stroke.getCMYK()[3] * 100. / 255.));
		printf(" Detect: (%d,%d,%d,%d)\n", (int)CMYK[0], (int)CMYK[1], (int)CMYK[2], (int)CMYK[3]);
		printf(" Differ: (%d,%d,%d,%d)\n", (int)(stroke.getCMYK()[0] * 100. / 255.) - (int)CMYK[0], (int)(stroke.getCMYK()[1] * 100. / 255.) - (int)CMYK[1],
			(int)(stroke.getCMYK()[2] * 100. / 255.) - (int)CMYK[2], (int)(stroke.getCMYK()[3] * 100. / 255.) - (int)CMYK[3]);
		captureDevice >> detectImg;
		imshow("Capture", detectImg);
		cvWaitKey(33);
		Sleep(33);
	}
}
void GoToPoint(float x, float y, float z, float theta_roll, float theta_pitch, float theta_yaw, float theta_arm){
	theta_roll /= 180.0;
	theta_roll *= pi_f;
	theta_pitch /= 180.0;
	theta_pitch *= pi_f;
	theta_yaw /= 180.0;
	theta_yaw *= pi_f;
	theta_arm /= 180.0;
	theta_arm *= pi_f;
	T << -cos(theta_pitch + theta_yaw), sin(theta_yaw), -sin(theta_pitch), x,
		sin(theta_yaw), cos(theta_roll + theta_yaw), -sin(theta_roll), y,
		sin(theta_pitch), -sin(theta_roll), -cos(theta_roll + theta_pitch), z,
		0.0f, 0.0f, 0.0f, 1.0f;
	Move_L_Abs(T, theta_arm);
	if (level != 0)
		idleDisplay();
}
void MoveRelative(float x, float y, float z, float r, float p, float yaw){
	t << x, y, z, r, p, yaw;
	Move_L_Rel(t, 0);
	while (MOVL){ system("cls"); DisplayLoop();  Sleep(33); }
}
void DipColor(float d){
	t << 0, 0, -d, 0, 0, 0;
	Move_L_Rel(t, 0);
	idleDisplay();

	t << 0, 0, d, 0, 0, 0;
	Move_L_Rel(t, 0);
	idleDisplay();
}
void MixColor(int mix_times){
	float d = 0.012;
	t << 0, 0, -d, 0, 0, 0;
	Move_L_Rel(t, 0);
	idleDisplay();

	t << 0, 0.005, 0, 0, 0, 0;
	Move_L_Rel(t, 0);
	idleDisplay();

	for (int i = 0; i < mix_times; i++){
		t << 0, -0.01, 0, 0, 0, 0;
		Move_L_Rel(t, 0);
		idleDisplay();
		t << 0, 0.01, 0, 0, 0, 0;
		Move_L_Rel(t, 0);
		idleDisplay();
	}

	t << 0, -0.005, 0, 0, 0, 0;
	Move_L_Rel(t, 0);
	idleDisplay();
}
vector<StrokeCluster> readFirstStroke(int & stroke_num, int &picture_id){
	//count how many files in drawPoints directory
	WIN32_FIND_DATA fd;
	HANDLE h = FindFirstFile(TEXT("drawPoints/fill*.txt"), &fd);
	if (h != INVALID_HANDLE_VALUE) {
		do {
			stroke_num++;
		} while (FindNextFile(h, &fd));
		FindClose(h);
	}
	h = FindFirstFile(TEXT("Image/take*.jpg"), &fd);
	if (h != INVALID_HANDLE_VALUE) {
		do {
			picture_id++;
		} while (FindNextFile(h, &fd));
		FindClose(h);
	}
	vector<StrokeCluster> firstDrawStrokes(stroke_num);

	// Read fill points
	vector<string> sep;
	for (int i = 0; i < stroke_num; i++){
		string file_num = int2str(i);
		string file_name = "drawPoints/fill";
		file_name.append(file_num);
		file_name.append(".txt");
		ifstream file(file_name);

		string str;
		// Read rgb and cmyk
		getline(file, str);
		sep = split(str, ' ');
		Vec3b rgb = Vec3b(stoi(sep[0]), stoi(sep[1]), stoi(sep[2]));
		getline(file, str);
		sep = split(str, ' ');
		Vec4f cmyk = Vec4f(stof(sep[0]), stof(sep[1]), stof(sep[2]), stof(sep[3]));

		while (getline(file, str))
		{
			sep = split(str, ' ');
			Point start = Point(stoi(sep[0]), stoi(sep[1]));
			Point end = Point(stoi(sep[2]), stoi(sep[3]));
			firstDrawStrokes[i].addStroke(Stroke(rgb, cmyk, start, end, 5));
		}
	}
	return firstDrawStrokes;
}
void DrawStroke(Stroke stroke){
	float scale = paperSize / imageSize / 100.0;
	float y1 = (-1) * (stroke.getPoint(0).x - 200.0) * scale + canvas_center.y;
	float y2 = (-1) * (stroke.getPoint(1).x - 200.0) * scale + canvas_center.y;
	float x1 = (-1) * (stroke.getPoint(0).y - 200.0) * scale + canvas_center.x;
	float x2 = (-1) * (stroke.getPoint(1).y - 200.0) * scale + canvas_center.x;
	GoToPoint(x1, y1, board_touch + view_dz, 0, 0, 0, 0);
	GoToPoint(x2, y2, board_touch + view_dz, 0, 0, 0, 0);
}
void setDefaultArmSpeed(float percentage)
{
	Ang_Vel_limit = (0.01f * 100 * deg2rad*RVmax / 1000.0f) *SAMPLING_TIME_C * percentage;
	Ang_Acc_limit = ((0.01f * 100 * deg2rad*RAmax / 1000.0f) *SAMPLING_TIME_C) / 1000 * SAMPLING_TIME_C * percentage;
	Ang_Dec_limit = Ang_Acc_limit * percentage;
	Lin_Vel_limit = (0.01f * 160 * 0.001f*LVmax / 1000.0f) *SAMPLING_TIME_C * percentage;
	Lin_Acc_limit = ((0.01f * 320 * 0.001f*LAmax / 1000.0f) *SAMPLING_TIME_C) / 1000 * SAMPLING_TIME_C * percentage;
	Lin_Dec_limit = Lin_Acc_limit * percentage;
	Jn_Vel_limit = (0.01f * 25 * deg2rad*RVmax / 1000.0f) *SAMPLING_TIME_C * percentage;
	Jn_Acc_limit = ((0.01f * 25 * deg2rad*RAmax / 1000.0f) *SAMPLING_TIME_C) / 1000 * SAMPLING_TIME_C * percentage;
	Jn_Dec_limit = Jn_Acc_limit * percentage;
}