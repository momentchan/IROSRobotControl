#include "LuoLitaArm.h"
#include "FuncDeclaration.h"
using namespace std;
using namespace Eigen;
extern int mode_display;
// Global variable
// Timer
void RTFCNDCL TimerHandler1(void * nContext);
vector<StrokeCluster> firstDrawStrokes;
vector<StrokeCluster> StrokeClusters;

// Camera
VideoCapture captureDevice;
Mat detectImg;
double canvasFocus = 10;
double drawFocus = 15;
int picture_id = 0;

// View
int corner_x = 321, corner_y = 93, w = 289, h = 289;
int view_id = 0;
Rect viewWindow;
Rect canvasView = Rect(314, 100, 289, 289);//Rect(259, 81, 306, 306);
Rect drawView = Rect(300, 417, 25, 25);//Rect(283, 344, 25, 25); // short: Rect(301, 381, 25, 25); 

// Visual Feedback 
float iter = 0;
Mat targetImg = imread("apple.jpg");
Mat edgeMap, angles;


// Mix
Vec4f CMYK;
Stroke stroke;
int cluster_num = 0;
int cluster_id = 0;
int stroke_id = 0;
int mix_times = 2;
char mix_color;
float mix_id = 0;
float mix_dx = 0.03;
float mix_d = 0.007;
float level = 0;

// Position
float board_touch = -0.124; //-0.138; // short: -0.19 long:-0.145  -0.111 -0.131
float color_touch = -0.118;
float dip_depth = 0.005;//0.005;
float view_dx = -0.015; // short: -0.02
float view_dz = 0.023;//0.015; //short: 0.01;
Point3f pos_C = Point3f(0.65, 0.17, board_touch + view_dz); // z = -0.18 
Point3f pos_M = Point3f(0.61, 0.17, board_touch + view_dz);
Point3f pos_Y = Point3f(0.57, 0.17, board_touch + view_dz);
Point3f pos_K = Point3f(0.53, 0.17, board_touch + view_dz);
Point3f pos_mix = Point3f(0.65, 0.12, board_touch + view_dz);
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
bool swithColor = true;
char DrawMode = 'c'; // c: capture only   m: mix   d: draw
bool iterDone = true;
bool firstStroke = true;
static void onMouse(int event, int x, int y, int f, void* userdata){
	if (event == CV_EVENT_RBUTTONDOWN){
		mousePosition.x = x;
		mousePosition.y = y;
	}
}
void KeyboardControl(){
	switch (kbCmd){
		// Speed Control
		case '-':
			if (speed>0.2) speed -= 0.1f;
			setDefaultArmSpeed(speed);
			break;
		case '+':
			if (speed<1.2) speed += 0.1f;
			setDefaultArmSpeed(speed);
			break;
		// Move Control
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
			cout << "\n Input step move \n";
			cin >> step_move;
			break;
		// View Control
		case 'o':
			GoToPoint(0.5f, 0, 0, 0, 0, 0, 0);
			SetCamera("canvas");
			break;
		case 'v':
			cout << "\n Input view color id \n";
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
		// Finger Control
		case 'n':
			finger.move(80);
			break;
		// Camera Control
		case 'c': {
			string fileName = outputFileName("Image/take", picture_id, ".jpg");
			imwrite(fileName, detectImg);
			picture_id++;
			break;
		}
		case 'p':
			cout << "Camera focus on: " << captureDevice.get(CV_CAP_PROP_FOCUS) << endl;
			PAUSE
			break;
		case 'm':		// change mode
			cout << "\n Input drawing mode \n";
			cin >> DrawMode;
			break;
	}
	kbCmd = ' ';
}
Mat largeCanvas;
void ModeTransition(){
	switch (DrawMode){
		// Camera View Mode
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
		// Mix color Mode
		case 'm':{
			SetCamera("draw");
			// Go to mix position to check color
			if (swithColor){
				swithColor = false;
			}
			GoToPoint(pos_mix.x - mix_id * mix_dx + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
			bool mixed = false;
			while (true){
				system("cls");
				DisplayLoop();
				captureDevice >> detectImg;
				DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
				float dip_z = (board_touch + view_dz - color_touch) + level / 100.0 * dip_depth;

				if (mix_color == 'N'){
					if (!mixed){
						GoToPoint(pos_mix.x, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
						// Mix color
						MixColor(view_dz, mix_times);
						GoToPoint(pos_mix.x, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
					}
					if (firstStroke)
						DrawMode = 'd';
					else
						DrawMode = 'f';
					break;
				}
				else{
					// Go to upper of color 
					if (mix_color == 'C')	GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);
					else if (mix_color == 'M')	GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
					else if (mix_color == 'Y')	GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
					else if (mix_color == 'K')	GoToPoint(pos_K.x, pos_K.y, pos_K.z, 0, 0, 0, 0);
					// Dip color
					DipColor(dip_z);
					GoToPoint(pos_mix.x, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
					// Mix color
					MixColor(view_dz, mix_times);
					GoToPoint(pos_mix.x + view_dx, pos_mix.y, pos_mix.z, 0, 0, 0, 0);
					mixed = true;
				}
				if (_kbhit()) kbCmd = _getche();
				if (kbCmd == kb_ESC) {
					DrawMode = 'c';
					break;
				}
				//Sleep(33);
			}
			break;
		}
		// Draw First Layer Mode
		case 'd':
			SetCamera("draw");
			if (stroke_id < firstDrawStrokes[cluster_id].getNum()){
				stroke = firstDrawStrokes[cluster_id].getStroke(stroke_id);
				// Draw stroke
				if (DrawStroke(stroke)){
					captureDevice >> detectImg;
					DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
					if (mix_color != 'N')
						DrawMode = 'm';
				}
				stroke_id++;
			}
			else{
				cluster_id++;
				printf("\n Region %d is finished! \n", cluster_id);
				PAUSE
				mix_id++;
				swithColor = true;
				if (cluster_id < cluster_num){
					stroke_id = 0;
					stroke = firstDrawStrokes[cluster_id].getStroke(stroke_id);

					captureDevice >> detectImg;
					DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
					if (mix_color != 'N')
						DrawMode = 'm';
				}
				else{
					printf("\n First Layer stroke finished. \n");
					Sleep(1000); 
					DrawMode = 'c';
					firstStroke = false;
				}
			}
			break;
		// Visual Feedback Mode
		case 'f':{
			if (iter < ITERATION){
				// Generate Strokes
				if (iterDone){
					GoToPoint(0.5f, 0, 0, 0, 0, 0, 0);
					SetCamera("canvas");
#if SIMULATION
					if (iter == 0){
						captureDevice >> detectImg;
						detectImg.copyTo(largeCanvas);
					}
					else
						largeCanvas.copyTo(detectImg);
					Mat canvas = detectImg(Rect(corner_x, corner_y, w, h));
					resize(canvas, canvas, Size(imageSize, imageSize));
					/*if (iter == 0){
						for (int c = 0; c < firstDrawStrokes.size(); c++){
							int strokeNum = firstDrawStrokes[c].getNum();
							Vec4f CMYK = firstDrawStrokes[c].getColor();
							for (int s = 0; s < strokeNum; s++){
								firstDrawStrokes[c].getStroke(s).drawOnCanvas(canvas);
								Mat resizeCanvas;
								resize(canvas, resizeCanvas, Size(w, h));
								resizeCanvas.copyTo(largeCanvas(Rect(corner_x, corner_y, w, h)));
								imshow("Simulation", largeCanvas);
								waitKey(10);
							}
						}
					}*/


					vector<pair <Point, float>> drawPoints;
					colorDiffer(targetImg, canvas, drawPoints, iter);

					// Generate Strokes
					StrokeClusters = StrokesGeneration(targetImg, canvas, drawPoints, edgeMap, angles, iter);
					// Draw on canvas
					printf("Drawing iteration : %d \n", (int)iter);
					for (int c = 0; c < StrokeClusters.size(); c++){
						int strokeNum = StrokeClusters[c].getNum();
						Vec4f CMYK = StrokeClusters[c].getColor();
						//printf("  # of stroke in cluster %d : %d   (%d, %d, %d, %d) %d \n",
						//	c, strokeNum, (int)CMYK[0], (int)CMYK[1], (int)CMYK[2], (int)CMYK[3], StrokeClusters[c].getMaxInfo().first);
						for (int s = 0; s < strokeNum; s++){
							StrokeClusters[c].getStroke(s).drawOnCanvas(canvas, edgeMap);
							Mat resizeCanvas;
							resize(canvas, resizeCanvas, Size(w, h));
							resizeCanvas.copyTo(largeCanvas(Rect(corner_x, corner_y, w, h)));
							imshow("Simulation", largeCanvas);
							waitKey(10);
						}
					}
#else//Simulation
					captureDevice >> detectImg;
					detectImg.copyTo(largeCanvas);

					Mat canvas = detectImg(Rect(corner_x, corner_y, w, h));
					resize(canvas, canvas, Size(imageSize, imageSize));

					vector<pair <Point, float>> drawPoints;
					colorDiffer(targetImg, canvas, drawPoints, iter);
					StrokeClusters = StrokesGeneration(targetImg, canvas, drawPoints, edgeMap, angles, iter);
					cluster_num = StrokeClusters.size();
					cluster_id = 0;
					stroke_id = 0;
					iterDone = false;

					// Draw on canvas
					printf("\n Drawing iteration : %d \n", (int)iter);
					for (int c = 0; c < StrokeClusters.size(); c++){
						int strokeNum = StrokeClusters[c].getNum();
						Vec4f CMYK = StrokeClusters[c].getColor();
						//printf("  # of stroke in cluster %d : %d   (%d, %d, %d, %d) %d \n",
						//	c, strokeNum, (int)CMYK[0], (int)CMYK[1], (int)CMYK[2], (int)CMYK[3], StrokeClusters[c].getMaxInfo().first);
						for (int s = 0; s < strokeNum; s++){
							StrokeClusters[c].getStroke(s).drawOnCanvas(canvas, edgeMap);
							Mat resizeCanvas;
							resize(canvas, resizeCanvas, Size(w, h));
							resizeCanvas.copyTo(largeCanvas(Rect(corner_x, corner_y, w, h)));
							imshow("Simulation", largeCanvas);
							waitKey(10);
						}
					}
				}

				if (stroke_id < StrokeClusters[cluster_id].getNum()){
					stroke = StrokeClusters[cluster_id].getStroke(stroke_id);
					// Draw stroke
					if (DrawStroke(stroke)){
						captureDevice >> detectImg;
						DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
						if (mix_color != 'N')
							DrawMode = 'm';
					}
					stroke_id++;
				}
				else{
					cluster_id++;
					printf(" \n Region %d is finished! \n", cluster_id);
					Sleep(1000);
					swithColor = true;
					if (cluster_id < cluster_num){
						stroke_id = 0;
						stroke = StrokeClusters[cluster_id].getStroke(stroke_id);

						captureDevice >> detectImg;
						DisplayInfo(detectImg, viewWindow, stroke, mix_color, level, CMYK);
						if (mix_color != 'N')
							DrawMode = 'm';
					}
					else{
						printf("\n Iteration %d is finished. \n ", iter);
						iterDone = true;
						PAUSE
					}
#endif
				}
			}
			else{
				printf(" \n Drawing is finish! \n ");
				PAUSE
				DrawMode = 'c';
			}
			break;
		}
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
	Init_IMPCard();
	Close_IMPCard();
	Sleep(1000);
	// Read first draw strokes
	firstDrawStrokes = readFirstStroke(cluster_num, picture_id);
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
	
	EMapConstruct(targetImg, edgeMap, angles);

	// Initialize Camera
	captureDevice.open(0);
	if (!captureDevice.isOpened()){
		cout << "Camera not found!";
		PAUSE
	}
	// Initialize Finger Pose
	//initFinger();

	stroke = firstDrawStrokes[0].getStroke(0);
	while (true)
	{
		// Get keyboard command
		if (_kbhit()) kbCmd = _getche();
		if (kbCmd == kb_ESC) break;

		system("cls");
		DisplayLoop();
		printf("\n  Moving speed: %.2f	 Drawing Mode: %c \n ", speed, DrawMode);
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

