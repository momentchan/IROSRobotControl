#include "LuoLitaArm.h"
#include "FuncDeclaration.h"

using namespace std;
using namespace Eigen;

//-----------------------------
//------Global Variable--------
//-----------------------------
extern int mode_display;

// Timer
void RTFCNDCL TimerHandler1(void * nContext);
vector<StrokeCluster> firstDrawStrokes;

// Camera
VideoCapture captureDevice;
Mat detectImg;
double canvasFocus = 10;
double drawFocus = 15;
int picture_id = 0;

// View
int corner_x = 261, corner_y = 71, w = 283, h = 283;
int view_id = 0;
Rect viewWindow;
Rect canvasView = Rect(259, 81, 306, 306); 
Rect drawView = Rect(283, 344, 25, 25); // short: Rect(301, 381, 25, 25); 

// Mix
Vec4f CMYK;
Stroke stroke;
int stroke_num = 0;
int stroke_id = 0;
int mix_times = 2;
char mix_color;
float mix_id = 0;
float mix_dx = 0.03;
float level = 0;

// Position
float board_touch = -0.145; // short: -0.19 long:-0.145
float dip_depth = 0.005;
float view_dx = -0.015; // short: -0.02
float view_dz = 0.015; //short: 0.01;
Point3f pos_C = Point3f(0.65, 0.17, board_touch + view_dz); // z = -0.18 
Point3f pos_M = Point3f(0.61, 0.17, board_touch + view_dz);
Point3f pos_Y = Point3f(0.57, 0.17, board_touch + view_dz);
Point3f pos_K = Point3f(0.53, 0.17, board_touch + view_dz);
Point3f pos_mix = Point3f(0.65, 0.13, board_touch + view_dz);
Point2f canvas_center = Point2f(0.6, -0.1);
Point mousePosition = Point(0, 0);
Vector6f t;
Matrix4f T;

// Move
float speed = 0.5f;
float step_move = 0.005f;

// Draw Point Information
float paperSize = 20;
float imageSize = 400;

// Other
Finger finger;
char DrawMode = 'c'; // c: capture only   m: mix   d: draw
bool loopDone = false;

// Function
void initFinger(){
	//Initial finger pose
	finger.init();
	finger.open();
	finger.setMode('p');
	finger.move(80);
}
void CreatRectangle(int, void*){
	viewWindow = Rect(corner_x, corner_y, w, h);
}
static void onMouse(int event, int x, int y, int f, void* userdata){
	if (event == CV_EVENT_RBUTTONDOWN){
		mousePosition.x = x;
		mousePosition.y = y;
	}
}


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
			MoveRelative(step_move, 0, 0, 0, 0, 0);
			break;
		case 'd':
			MoveRelative(-step_move, 0, 0, 0, 0, 0);
			break;
		case 'w':
			MoveRelative(0, -step_move, 0, 0, 0, 0);
			break;
		case 's':
			MoveRelative(0, step_move, 0, 0, 0, 0);
			break;
		case 'r':
			MoveRelative(0, 0, step_move, 0, 0, 0);
			break;
		case 'f':
			MoveRelative(0, 0, -step_move, 0, 0, 0);
			break;
		case 'q':
			cin >> step_move;
			break;
		case 'o':
			GoToPoint(0.5f, 0, 0, 0, 0, 0, 0);
			captureDevice.set(CV_CAP_PROP_FOCUS, canvasFocus);
			corner_x = canvasView.x, corner_y = canvasView.y, w = canvasView.width, h = canvasView.height;
			viewWindow = Rect(corner_x, corner_y, w, h);
			break;
		case 'v':
			cin >> view_id;
			corner_x = drawView.x, corner_y = drawView.y, w = drawView.width, h = drawView.height;
			if (view_id == 0){
				GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);
			}
			else if (view_id == 1){
				GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
			}
			else if (view_id == 2){
				GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
			}
			else if (view_id == 3){
				GoToPoint(pos_K.x, pos_K.y, pos_K.z, 0, 0, 0, 0);
			}
			break;
		case 'c':
			fileName = outputFileName("Image/take", picture_id, ".jpg");
			imwrite(fileName, detectImg);
			picture_id++;
			break;
		case 'm':		// change mode
			cin >> DrawMode;
			break;
		case 'n':
			finger.move(80);
			break;
		case '2':
			stroke = firstDrawStrokes[0].getStroke(stroke_id);
			DrawStroke(stroke);
			stroke_id++;
			break;
		case '/':
			cout << captureDevice.get(CV_CAP_PROP_FOCUS);
			PAUSE
			break;

	}
	kbCmd = ' ';
}
void ModeTransition(){
	switch (DrawMode){
		case 'm':
			captureDevice.set(CV_CAP_PROP_FOCUS, drawFocus);
			corner_x = drawView.x, corner_y = drawView.y, w = drawView.width, h = drawView.height;
			viewWindow = Rect(corner_x, corner_y, w, h);
			destroyWindow("Rectangle");
			
			// Go to mix position to check color
			pos_mix.x -= mix_id * mix_dx;
			GoToPoint(pos_mix.x + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);

			while (true){
				system("cls");
				DisplayLoop();
				captureDevice >> detectImg;
				DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
				float dip_z = (1.0 + level / 100.0) * dip_depth;

				if (mix_color == 'N'){
					DrawMode = 'c';
					break;
				}
				else{
					// Go to upper
					if		(mix_color == 'C')	GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);	
					else if (mix_color == 'M')	GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
					else if (mix_color == 'Y')	GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
					else if (mix_color == 'K')	GoToPoint(pos_K.x, pos_K.y, pos_K.z, 0, 0, 0, 0);
					// Dip color
					DipColor(dip_z);
					GoToPoint(pos_mix.x, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
					// Mix color
					MixColor(mix_times);
					GoToPoint(pos_mix.x + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
				}
				if (_kbhit()) kbCmd = _getche();
				if (kbCmd == kb_ESC) {
					DrawMode = 'c';
					break;
				}
				Sleep(33);
			}
		break;
		case 'c':
			captureDevice >> detectImg;
			namedWindow("Rectangle");
			createTrackbar("X", "Rectangle", &corner_x, 640, CreatRectangle);
			createTrackbar("Y", "Rectangle", &corner_y, 480, CreatRectangle);
			createTrackbar("W", "Rectangle", &w, 640, CreatRectangle);
			createTrackbar("H", "Rectangle", &h, 480, CreatRectangle);
			CreatRectangle(0, 0);
			DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
			break;
		case 'd':
			captureDevice.set(CV_CAP_PROP_FOCUS, drawFocus);
			corner_x = drawView.x, corner_y = drawView.y, w = drawView.width, h = drawView.height;
			viewWindow = Rect(corner_x, corner_y, w, h);
			destroyWindow("Rectangle");

			// Go to mix position to check color
			pos_mix.x -= mix_id * mix_dx;
			GoToPoint(pos_mix.x + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);

			while (true){
				system("cls");
				DisplayLoop();
				captureDevice >> detectImg;
				DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
				float dip_z = (1.0 + level / 100.0) * dip_depth;

				if (mix_color == 'N'){
					DrawMode = 'c';
					break;
				}
				else{
					// Go to upper
					if (mix_color == 'C')	GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);
					else if (mix_color == 'M')	GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
					else if (mix_color == 'Y')	GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
					else if (mix_color == 'K')	GoToPoint(pos_K.x, pos_K.y, pos_K.z, 0, 0, 0, 0);
					// Dip color
					DipColor(dip_z);
					GoToPoint(pos_mix.x, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
					// Mix color
					MixColor(mix_times);
					GoToPoint(pos_mix.x + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
				}
				if (_kbhit()) kbCmd = _getche();
				if (kbCmd == kb_ESC) {
					DrawMode = 'c';
					break;
				}
				Sleep(33);
			}
			break;
		case 'f':
			captureDevice.set(CV_CAP_PROP_FOCUS, canvasFocus);
			corner_x = canvasView.x, corner_y = canvasView.y, w = canvasView.width, h = canvasView.height;
			viewWindow = Rect(corner_x, corner_y, w, h);
			destroyWindow("Rectangle");

			// Go to mix position to check color
			pos_mix.x -= mix_id * mix_dx;
			GoToPoint(pos_mix.x + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);

			while (true){
				system("cls");
				DisplayLoop();
				captureDevice >> detectImg;
				DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
				float dip_z = (1.0 + level / 100.0) * dip_depth;

				if (mix_color == 'N'){
					DrawMode = 'c';
					break;
				}
				else{
					// Go to upper
					if (mix_color == 'C')	GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);
					else if (mix_color == 'M')	GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
					else if (mix_color == 'Y')	GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
					else if (mix_color == 'K')	GoToPoint(pos_K.x, pos_K.y, pos_K.z, 0, 0, 0, 0);
					// Dip color
					DipColor(dip_z);
					GoToPoint(pos_mix.x, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
					// Mix color
					MixColor(mix_times);
					GoToPoint(pos_mix.x + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
				}
				if (_kbhit()) kbCmd = _getche();
				if (kbCmd == kb_ESC) {
					DrawMode = 'c';
					break;
				}
				Sleep(33);
			}
			break;
		case 'b':
			finger.close();
			break;
		case 'n':
			finger.move(80);
			break;

	}
	kbCmd = ' ';
}

int main(int argc, char **argv, char **envp)
{
	// Read first draw strokes
	firstDrawStrokes = readFirstStroke(stroke_num, picture_id);
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
	kbCmd = 'j';
	MainLoop_keyboard();
	setDefaultArmSpeed(speed);
#endif
	kbCmd = ' ';
	mode_display = 0;

	// Initialize Camera
	captureDevice.open(0);
	if (!captureDevice.isOpened()){
		cout << "Camera not found!";
		return 0;
	}
	// Initialize Finger Pose
	//initFinger();

	stroke = firstDrawStrokes[0].getStroke(0);
	
	while (!loopDone)
	{
		// Get keyboard command
		if (_kbhit()) kbCmd = _getche();
		if (kbCmd == kb_ESC) break;

		system("cls");
		DisplayLoop();
		cout << endl << "  Moving speed: " << speed << "   ";
		cout << "Drawing Mode = " << DrawMode << endl;
		setMouseCallback("Capture", onMouse, 0);
		KeyboardControl();
		ModeTransition();
		Sleep(33);
	}
	destroyAllWindows();
	printf("\n Drawing is finish. \n");
	Sleep(500);

	kbCmd = ' ';

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
		DisplayLoop();
	}

	LitaHand.GripperGoHome();
	Sleep(100);
	if (!RtDeleteTimer(hTimer1))
	{
		Close_IMPCard();
		ExitProcess(1);
	}
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

