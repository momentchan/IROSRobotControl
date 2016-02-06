#include "LuoLitaArm.h"
#include "FuncDeclaration.h"

using namespace std;
using namespace Eigen;
//-----------------------------
//------Global Variable--------
//-----------------------------

// function prototype for periodic timer function
void RTFCNDCL TimerHandler1(void * nContext);

VideoCapture captureDevice;
Mat detectImg;
Stroke stroke;
int pictureID = 0;

// Position information
int corner_x = 249, corner_y = 81, w = 306, h = 306;
float board_touch = -0.185;
float mix_z = board_touch + 0.02;
float view_z = board_touch + 0.08;
Point3f pos_C = Point3f(0.65, 0.167, mix_z);
Point3f pos_M = Point3f(0.61, 0.167, mix_z);
Point3f pos_Y = Point3f(0.57, 0.167, mix_z);
Point3f pos_K = Point3f(0.53, 0.167, mix_z);

Rect canvasView = Rect(259, 81, 306, 306);
Rect drawView = Rect(301, 381, 25, 25);
Rect viewWindow;


// Draw Point Information
float paperSize = 25;
float imageSize = 400;


// other initial parameter
float speed = 0.5f;
bool initialDone = false;

// angle and math definition
float pi_f = (float)M_PI;
int StrokeNum = 0;

// finger call out
Finger finger;
// Position matrix
Vector6f t;
Matrix4f T;

void initFinger(){
	//Initial finger pose
	finger.init();
	finger.open();
	finger.setMode('p');
	finger.move(80);
}

void CreatRectangle(int, void*){
	viewWindow = Rect(corner_x, corner_y, w, h);
	//rectangle(detectImg, Rect(corner_x, corner_y, w, h), (255, 0, 0), 2);
}
//static void onMouse(int event, int x, int y, int f, void* userdata){
//	if (event == CV_EVENT_RBUTTONDOWN){
//		mousePosition.x = x;
//		mousePosition.y = y;
//		rgb2cmyk(targetBGR, targetCMYK);
//		targetBGR = captureImage.at<Vec3b>(y, x);
//		rgb2cmyk(targetBGR, targetCMYK);
//	}
//}
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
	while (MOVL){ system("cls");	DisplayLoop(speed); }
}
void MoveRelative(float x, float y, float z, float r, float p, float yaw){
	t << x, y, z, r, p, yaw;
	Move_L_Rel(t, 0);
	while (MOVL){ system("cls");	DisplayLoop(speed); }
}
void dolikehuman(float X, float Y, float Z, float move_y, float move_z, float move_roll){
	GoToPoint(X, Y, Z - 0.12 + move_z, 0, 0, 0, 50);
	GoToPoint(X, Y - 0.02, Z - 0.12 + move_z, move_roll, 0, 0, 50);
	GoToPoint(X, Y - move_y, Z - 0.12 + move_z, move_roll, 0, 0, 50);
	GoToPoint(X, Y - move_y + 0.005, Z - 0.12 + move_z, move_roll, 0, 0, 50);
	GoToPoint(X, Y - move_y, Z - 0.12 + move_z, move_roll, 0, 0, 50);
	//GoToPoint(X,Y-move_y+0.005,Z-0.12+move_z,move_roll,0,0,50);
	GoToPoint(X, Y - move_y + 0.005 + 0.008, Z - 0.12 + move_z, move_roll, 0, 45, 50);
	GoToPoint(X, Y - move_y + 0.005 + 0.0035, Z - 0.12 + move_z, move_roll, 0, 45, 50);
	GoToPoint(X, Y - move_y + 0.005 + 0.008, Z - 0.12 + move_z, move_roll, 0, 45, 50);
	GoToPoint(X, Y - move_y + 0.005 + 0.0035, Z - 0.12 + move_z, move_roll, 0, 45, 50);
	GoToPoint(X, Y, Z, 0, 0, 0, 50);
}
void DipPen(float level){
	float d = 0.12 + (level / 100.0) * 2.0;
	t << 0, 0, -d, 0, 0, 0;
	Move_L_Rel(t, 0);
	while (MOVL){ system("cls");	DisplayLoop(speed); }

	t << 0, 0, d, 0, 0, 0;
	Move_L_Rel(t, 0);
	while (MOVL){ system("cls");	DisplayLoop(speed); }
}
int viewIndex = 0;
void KeyboardControl(){
	string fileName;
	switch (kbCmd){
		case '-':
			if (speed>0.2) speed -= 0.1f;
			setDefaultArmSpeed(speed);
			break;
		case '+':
			if (speed<1.2) speed += 0.1f;
			setDefaultArmSpeed(speed);
			break;
		case 'a':
			MoveRelative(0.01f, 0, 0, 0, 0, 0);
			break;
		case 'd':
			MoveRelative(-0.01f, 0, 0, 0, 0, 0);
			break;
		case 'w':
			MoveRelative(0, -0.01f, 0, 0, 0, 0);
			break;
		case 's':
			MoveRelative(0, 0.01f, 0, 0, 0, 0);
			break;
		case 'j':
			MoveRelative(0.05f, 0, 0, 0, 0, 0);
			break;
		case 'l':
			MoveRelative(-0.05f, 0, 0, 0, 0, 0);
			break;
		case 'i':
			MoveRelative(0, -0.05f, 0, 0, 0, 0);
			break;
		case 'k':
			MoveRelative(0, 0.05f, 0, 0, 0, 0);
			break;
		case 'r':
			MoveRelative(0, 0, 0.01f, 0, 0, 0);
			break;
		case 'f':
			MoveRelative(0, 0, -0.01f, 0, 0, 0);
			break;
		case 't':
			MoveRelative(0, 0, 0.05f, 0, 0, 0);
			break;
		case 'g':
			MoveRelative(0, 0, -0.05f, 0, 0, 0);
			break;
		case 'o':
			GoToPoint(0.5f, 0, 0, 0, 0, 0, 0);
			break;
		case 'p':
			GoToPoint(0.5f, 0, view_z, 0, 0, 0, 0);
			break;
		case 'v':
			cin >> viewIndex;
			corner_x = drawView.x, corner_y = drawView.y, w = drawView.width, h = drawView.height;
			if (viewIndex == 0){
				GoToPoint(pos_C.x, pos_C.y, view_z, 0, 0, 0, 0);
			}
			else if (viewIndex == 1){
				GoToPoint(pos_M.x, pos_M.y, view_z, 0, 0, 0, 0);
			}
			else if (viewIndex == 2){
				GoToPoint(pos_Y.x, pos_Y.y, view_z, 0, 0, 0, 0);
			}
			else if (viewIndex == 3){
				GoToPoint(pos_K.x, pos_K.y, view_z, 0, 0, 0, 0);
			}
			break;
		case 'c':
			fileName = outputFileName("Image/take", pictureID, ".jpg");
			imwrite(fileName, detectImg);
			pictureID++;
			break;

		/*case 'b':
			GraspDropPen(0.12f, 1);
			break;
		case 'v':
			GraspDropPen(0.12f, 1);
			break;
		case 'c':
			GraspDropPen(0.08f, 0);
			break;*/
		case 'n':
			finger.move(80);
			break;
		case '2':
			finger.close();
			break;
	}
	kbCmd = ' ';
}
void ModeTransition(char DrawMode){
	float tempX;
	float tempY;
	float tempZ;
	switch (DrawMode){
	case 'm':
		while (true){
			captureDevice >> detectImg;
			char colorMix;
			int level;
			DisplayInfo(detectImg, viewWindow, stroke, colorMix, level);

			if (colorMix == 'N'){
				DrawMode = 'D';
				break;
			}
			else{
				if (colorMix == 'C'){
					GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);
				}
				else if (colorMix == 'M'){
					GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
				}
				else if (colorMix == 'Y'){
					GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
				}
				else if (colorMix == 'K'){
					GoToPoint(pos_K.x, pos_K.y, pos_K.z, 0, 0, 0, 0);
				}
			}
			break;
		}
		break;
		case 'c':
			captureDevice >> detectImg;
			char colorMix;
			int level;
			namedWindow("Rectangle");
			createTrackbar("X", "Rectangle", &corner_x, 640, CreatRectangle);
			createTrackbar("Y", "Rectangle", &corner_y, 480, CreatRectangle);
			createTrackbar("W", "Rectangle", &w, 640, CreatRectangle);
			createTrackbar("H", "Rectangle", &h, 480, CreatRectangle);
			CreatRectangle(0, 0);
			DisplayInfo(detectImg, viewWindow, stroke, colorMix, level);
			break;
		case 'b':
			finger.close();
			break;
		case 'n':
			finger.move(80);
			break;
	}
}
vector<StrokeCluster> readFirstStroke(){
	//count how many files in drawPoints directory
	WIN32_FIND_DATA fd;
	HANDLE h = FindFirstFile(TEXT("drawPoints/fill*.txt"), &fd);
	if (h != INVALID_HANDLE_VALUE) {
		do {
			StrokeNum++;
		} while (FindNextFile(h, &fd));
		FindClose(h);
	}
	h = FindFirstFile(TEXT("Image/take*.jpg"), &fd);
	if (h != INVALID_HANDLE_VALUE) {
		do {
			pictureID++;
		} while (FindNextFile(h, &fd));
		FindClose(h);
	}
	vector<StrokeCluster> firstDrawStrokes(StrokeNum);

	// Read fill points
	vector<string> sep;
	for (int i = 0; i < StrokeNum; i++){
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
			//float y1 = (-1)*(stoi(sep[0]) - 200)*(float)(paperSize / imageSize) / 100;//+centerX;
			//float x1 = (stoi(sep[1]) - 200)*(float)(paperSize / imageSize) / 100;//+centerY;
			//float y2 = (-1)*(stoi(sep[2]) - 200)*(float)(paperSize / imageSize) / 100;//+centerX;
			//float x2 = (stoi(sep[3]) - 200)*(float)(paperSize / imageSize) / 100;//+centerY;
		}
	}
	return firstDrawStrokes;
}
#define ROBOT_ON 1
int main(int argc, char **argv, char **envp)
{
	vector<StrokeCluster> firstDrawStrokes = readFirstStroke();
#if ROBOT_ON
	init_LuoLita_1();
	LitaHand.GripperMove_Abs_To(75.0f, 1);

	// for periodic timer code
	LARGE_INTEGER  liPeriod_1ms;   // timer period
	HANDLE         hTimer1;     // timer handle
	liPeriod_1ms.QuadPart = 10000;
	Init_IMPCard();
	// Create a periodic timer
	if (!(hTimer1 = RtCreateTimer( NULL, 0, TimerHandler1, NULL, RT_PRIORITY_MAX, CLOCK_2)))
		ExitProcess(1);
	if (!RtSetTimerRelative(hTimer1, &liPeriod_1ms, &liPeriod_1ms))
		ExitProcess(1);

	init_LuoLita_2();
	LitaHand.GripperMove_Abs_To(55.0f, 200);
#endif
	///////////////////////////////////////
	kbCmd = 'j';
	MainLoop_keyboard();

	// Initialize Camera
	captureDevice.open(0);
	if (!captureDevice.isOpened()){
		cout << "Camera not found!";
		return 0;
	}
		
	// Initialize Finger Pose
	//initFinger();

	setDefaultArmSpeed(speed);
	kbCmd = ' ';
	char DrawMode = 'c'; // 0: idle 1: mix 2: draw

	stroke = Stroke(Vec3b(233,161,0), Vec4f(255,78,0,21), Point2f(0,0), Point2f(0,0), 2);
	while (!initialDone)
	{
		// Get keyboard command
		if (_kbhit()) kbCmd = _getche();
		if (kbCmd == kb_ESC) break;
		system("cls");
		DisplayLoop(speed);
		KeyboardControl();
		ModeTransition(DrawMode);
		Sleep(33);
	}

	destroyAllWindows();

	cout << endl << " Initialization is done . " << endl;
	Sleep(1000);
	//setDefaultArmSpeed(speed);

	kbCmd = ' ';
	//while (1)
	//{
	//	if (_kbhit())
	//		kbCmd = _getche();

	//	if (kbCmd == kb_ESC)
	//	{
	//		break;
	//	}
	//	ModeTransition();
	//	Sleep(29);
	//	system("cls");
	//	DisplayLoop(speed);
	//	cout << endl;
	//	cout << "x_axis_color: " << x_axis_color << " z_axis_color: " << z_axis_color << endl;
	//	cout << "z_axis_black: " << x_axis_black << " z_axis_black: " << z_axis_black << endl;
	//	cout << "Board Angle: " << tuneAngle << endl;
	//	cout << "FillIndex: " << fillIndex << "  OilIndex: " << oilIndex << "  ShadowIndex: " << shadowIndex << endl;
	//	cout << "BrushTouch: " << brushTouch << "  brushUp: " << brushUp << "  brushDistance: " << brushDistance << endl;
	//	cout << "roll angle: " << roll_ang << "  pitch angle: " << pitch_ang << "  yaw angle: " << yaw_ang << "  arm angle: " << arm_ang << endl;
	//}
#if ROBOT_ON
	LitaHand.GripperMove_Abs_To(75.0f, 200);
	ByeBye();
	while (1)
	{
		if (_kbhit())
		{
			kbCmd = _getch();
			cout << kbCmd << endl;
			if (kbCmd == kb_ESC)
			{
				break;
			}
		}
		Sleep(30);
		system("cls");
		DisplayLoop(speed);

	}

	LitaHand.GripperGoHome();
	Sleep(100);
	//

	//if(!RtDeleteTimer( hTimer2 ) )
	//{
	//       //RtWprintf(L"RtDeleteTimer error = %d\n",GetLastError());
	//	// TO DO:  your exception code here
	//       ExitProcess(1);
	//}
	//RtDeleteTimer( hTimer2 );

	if (!RtDeleteTimer(hTimer1))
	{
		//RtWprintf(L"RtDeleteTimer error = %d\n",GetLastError());
		// TO DO:  your exception code here
		Close_IMPCard();
		ExitProcess(1);
	}
	//RtDeleteTimer( hTimer1 );

	//Close_IMPCard();
	Close_IMPCard();
	Sleep(1000);

	OutputData();

	ExitProcess(0);
#endif
}
// main end

void RTFCNDCL TimerHandler1(PVOID context)
{
	// TO DO:  your timer handler code here

	ServoLoop();
}





string int2str(int i) {
	string s;
	stringstream ss(s);
	ss << i;
	return ss.str();
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