#include "LuoLitaArm.h"
#include "FuncDeclaration.h"
using namespace std;
using namespace Eigen;
extern int mode_display;
// Global variable
// Timer
void RTFCNDCL TimerHandler1(void * nContext);
vector<StrokeCluster> UPDrawStrokes;
vector<StrokeCluster> StrokeClusters;

// Camera
VideoCapture CaptureDevice;
Mat DetectImg;
double canvasFocus = 10;
double drawFocus = 15;
int picture_id = 0;

// View
int corner_x = 0, corner_y = 0, w = 470, h = 470;
int pigment_id = 0;
Rect viewWindow;
Rect canvasView = Rect(136, 0, 389, 455);
Rect drawView = Rect(300, 417, 25, 25);

// Visual Feedback 
float iter = 0;
Mat TargetImg = imread("pindar.jpg");
Mat edgeMap, angles;

// Mix
Vec4f CMYK;
Stroke stroke;
int cluster_num = 0;
int cluster_id = 0;
int stroke_id = 0;
char mix_color;
int mix_id = 0;
float mix_dx = 0.03;		// distance between mixing positions
float mix_d = 0.008;		// back and forth distance
float dip_level = 0;		// determine dipping depth %

// Theshold
float mixThres = 15.0;
float mixThresBlack = 20.0;
int ColorChangeThres = 80;	// change mixing position

// Position
float board_touch = -0.168;
float color_touch = -0.145;
float max_depth = 0.005;
float view_dx = -0.015; 
float view_dz = 0.04;
Point3f pos_C = Point3f(0.64, 0.31, board_touch + view_dz); 
Point3f pos_M = Point3f(0.595, 0.31, board_touch + view_dz);
Point3f pos_Y = Point3f(0.55, 0.31, board_touch + view_dz);
Point3f pos_W = Point3f(0.505, 0.31, board_touch + view_dz);
Point3f pos_K = Point3f(0.46, 0.31, board_touch + view_dz);
Point3f pos_mix = Point3f(0.64, 0.26, board_touch + view_dz);
Point2f canvas_center = Point2f(0.55, -0.05); 
Point mousePosition = Point(0, 0);		// detect color using mouse (pixel)
Vector6f t;
Matrix4f T;

// Move
float speed = 0.8f;
float step_move = 0.005f;

// Draw Point Information
float paperSize = 38;					// [cm]
float imageWidth = TargetImg.cols;
float imageHeight = TargetImg.rows;

// Clustering and Ordering
int colorClass = 0;						// the class of mixing colr (from 0 to 7)
int strokeCount = 0;					// counting the number of drawn strokes ( reset to 0 in every 3 strokes)
int totalnum = 0;						// total number of strokes to be drawn
int num = 0;							// the number of all drawn strokes
bool showMarker = true;					// show the markers on the screen for detection
ofstream outputFile;

// Others
Finger finger;
char DrawMode = 'c';					// c: capture only   m: mix   d: draw  f: feedback
bool switchColor = true;
bool iterDone = true;
bool UPstroke = false;					// underpainting strokes
bool overlay = false;					// overlay original picture on canvas
bool startF = false;					// start feedback loop
vector<Vec4f> mixHistory;				// record mixed colors info
Vec4f lastColor;						// last mixed color


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
			GoToPoint(0.46, -0.05, 0.11, 0, 0, 0, 0);
			SetCamera("canvas");
			break;
		case 't':  // tuning the plan position
			GoToPoint(0.7f, 0.25, board_touch + 0.01, 0, 0, 0, 0);
			break;
		case 'v':
			cout << "\n Input view color id \n";
			cin >> pigment_id;
			corner_x = drawView.x, corner_y = drawView.y, w = drawView.width, h = drawView.height;
			if (pigment_id == 0){
				GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);
			}
			else if (pigment_id == 1){
				GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
			}
			else if (pigment_id == 2){
				GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
			}
			else if (pigment_id == 3){
				GoToPoint(pos_W.x, pos_W.y, pos_W.z, 0, 0, 0, 0);
			}
			else if (pigment_id == 4){
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
			imwrite(fileName, DetectImg);
			picture_id++;
			break;
		}
		case 'p':
			cout << "Camera focus on: " << CaptureDevice.get(CV_CAP_PROP_FOCUS) << endl;
			PAUSE
			break;
		case 'm':		// change mode
			cout << "\n Input drawing mode \n";
			cin >> DrawMode;
			break;
		case 'z':
			cout << board_touch << endl;
			cin >> board_touch;
			cout << board_touch << endl;
			PAUSE
			color_touch = board_touch + 0.023;
			pos_C = Point3f(0.64, 0.31, board_touch + view_dz);
			pos_M = Point3f(0.595, 0.31, board_touch + view_dz);
			pos_Y = Point3f(0.55, 0.31, board_touch + view_dz);
			pos_W = Point3f(0.505, 0.31, board_touch + view_dz);
			pos_K = Point3f(0.46, 0.31, board_touch + view_dz);
			pos_mix = Point3f(0.64, 0.26, board_touch + view_dz);
			break;
		case 'l':
			overlay = !overlay;
			break;
		case 'k':
			showMarker = !showMarker;
			break;
	}
	kbCmd = ' ';
}
Mat largeCanvas;
void ModeTransition(){
	switch (DrawMode){
		// Camera View Mode
		case 'c':
			CaptureDevice >> DetectImg;
			namedWindow("Rectangle");
			createTrackbar("X", "Rectangle", &corner_x, 640, CreatRectangle);
			createTrackbar("Y", "Rectangle", &corner_y, 480, CreatRectangle);
			createTrackbar("W", "Rectangle", &w, 640, CreatRectangle);
			createTrackbar("H", "Rectangle", &h, 480, CreatRectangle);
			CreatRectangle(0, 0);
			VisualFeedback(DetectImg, viewWindow, stroke, mix_color, dip_level, CMYK);
			break;
		// Mix color Mode
		case 'm':{
			SetCamera("draw");
			// Go to mix position to check color
			if (switchColor){
				switchColor = false;
			}
			GoToPoint(pos_mix.x - float(mix_id % 5) * mix_dx + view_dx, pos_mix.y -  (mix_id / 5) * mix_dx, pos_mix.z, 0, 0, 0, 0);
			bool mixed = false;
			int repeat_times = 0;
			while (true){
				repeat_times++;
				system("cls");
				DisplayLoop();
				CaptureDevice >> DetectImg;
				VisualFeedback(DetectImg, viewWindow, stroke, mix_color, dip_level, CMYK);
				lastColor = CMYK;
				float dip_z = (board_touch + view_dz - color_touch) + dip_level / 100.0 * max_depth;

				// Mix white
				if (stroke.getCMYK()[0] < 50 && stroke.getCMYK()[1] < 50 && stroke.getCMYK()[2] < 50 && stroke.getCMYK()[3] < 50){
					GoToPoint(pos_W.x, pos_W.y, pos_W.z, 0, 0, 0, 0);
					// Dip color
					DipColor(dip_z);
					if (UPstroke)
						DrawMode = 'd';
					else
						DrawMode = 'f';
					break;
				}
				else{
					if (mix_color == 'N' || repeat_times>9){
						outputFile << stroke.getCMYK()[0] << " " << stroke.getCMYK()[1] << " " << stroke.getCMYK()[2] << " " << stroke.getCMYK()[3] << " ";
						outputFile << CMYK[0] << " " << CMYK[1] << " " << CMYK[2] << " " << CMYK[3] << " " << repeat_times << endl;
						if (!mixed){
							GoToPoint(pos_mix.x - float(mix_id % 5) * mix_dx, pos_mix.y - (mix_id / 5) * mix_dx, pos_mix.z, 0, 0, 0, 0);
							// Mix color
							MixColor(view_dz, MIXTIMES);
							GoToPoint(pos_mix.x - float(mix_id % 5) * mix_dx, pos_mix.y - (mix_id / 5) * mix_dx, pos_mix.z, 0, 0, 0, 0);
						}
						if (UPstroke)
							DrawMode = 'd';
						else
							DrawMode = 'f';
						break;
					}
					else{
						// Go to upper of color 
						if (mix_color == 'C')	GoToPoint(pos_C.x, pos_C.y, pos_C.z, 0, 0, 0, 0);
						if (mix_color == 'M')	GoToPoint(pos_M.x, pos_M.y, pos_M.z, 0, 0, 0, 0);
						if (mix_color == 'Y')	GoToPoint(pos_Y.x, pos_Y.y, pos_Y.z, 0, 0, 0, 0);
						if (mix_color == 'K')	GoToPoint(pos_K.x, pos_K.y, pos_K.z, 0, 0, 0, 0);
						if (mix_color == 'W')	GoToPoint(pos_W.x, pos_W.y, pos_W.z, 0, 0, 0, 0);
						// Dip color
						DipColor(dip_z);
						GoToPoint(pos_mix.x - float(mix_id % 5) * mix_dx, pos_mix.y - (mix_id / 5) * mix_dx, pos_mix.z, 0, 0, 0, 0);
						// Mix color
						MixColor(view_dz, MIXTIMES);
						GoToPoint(pos_mix.x - float(mix_id % 5) * mix_dx + view_dx, pos_mix.y - (mix_id / 5) * mix_dx, pos_mix.z, 0, 0, 0, 0);
						mixed = true;
					}
					if (_kbhit()) kbCmd = _getche();
					if (kbCmd == 'b') {
						DrawMode = 'c';
						break;
					}
				}
				//Sleep(33);
			}
			break;
		}
		// Draw First Layer Mode
		case 'd':
			SetCamera("draw");
			if (stroke_id < UPDrawStrokes[cluster_id].getNum()){
				stroke = UPDrawStrokes[cluster_id].getStroke(stroke_id);
				// Draw stroke
				if (DrawStroke(stroke)){
					stroke_id++;
					strokeCount++;
					if (stroke_id < UPDrawStrokes[cluster_id].getNum()){
						if (strokeCount % 2 == 0){
							CaptureDevice >> DetectImg;
							VisualFeedback(DetectImg, Rect(306, 442, 25, 25), stroke, mix_color, dip_level, CMYK);
							if (mix_color != 'N')
								DrawMode = 'm';
						}
					}
				}
				else
					stroke_id++;
			}
			else{
				cluster_id++;
				printf("\n Region %d is finished! \n", cluster_id);
				PAUSE
				mix_id++;
				switchColor = true;
				if (cluster_id < cluster_num){
					stroke_id = 0;
					stroke = UPDrawStrokes[cluster_id].getStroke(stroke_id);
					DrawMode = 'm';
				}
				else{
					printf("\n First Layer stroke finished. \n");
					Sleep(1000); 
					DrawMode = 'c';
					UPstroke = false;
				}
			}
			break;
		// Visual Feedback Mode
		case 'f':{
			if (iter < ITERATION){
				// Generate Strokes
				if (iterDone){
						SetCamera("canvas");
#if SIMULATION
						if (iter == 0){
							CaptureDevice >> DetectImg;
							DetectImg.setTo(Scalar(255, 255, 255));
							DetectImg.copyTo(largeCanvas);
						}
						else
							largeCanvas.copyTo(DetectImg);
						Mat canvas = DetectImg(Rect(corner_x, corner_y, w, h));
						resize(canvas, canvas, Size(TargetImg.size()));
						if (iter == 0){
							for (int c = 0; c < UPDrawStrokes.size(); c++){
								int strokeNum = UPDrawStrokes[c].getNum();
								Vec4f CMYK = UPDrawStrokes[c].getColor();
								for (int s = 0; s < strokeNum; s++){
									UPDrawStrokes[c].getStroke(s).drawOnCanvas(canvas);
									Mat resizeCanvas;
									resize(canvas, resizeCanvas, Size(w, h));
									resizeCanvas.copyTo(largeCanvas(Rect(corner_x, corner_y, w, h)));
									imshow("Simulation", largeCanvas);
									waitKey(10);
								}
							}
							imwrite("image/stage0.jpg", largeCanvas);
						}


						vector<pair <Point, float>> drawPoints;
						colorDiffer(TargetImg, canvas, drawPoints, iter);

						// Generate Strokes
						StrokeClusters = StrokesGeneration(TargetImg, canvas, drawPoints, edgeMap, angles, iter);
						// Draw on canvas
						printf("Drawing iteration : %d \n", (int)iter);
						for (int c = 0; c < StrokeClusters.size(); c++){
							int strokeNum = StrokeClusters[c].getNum();
							Vec4f CMYK = StrokeClusters[c].getColor();
							printf("  # of stroke in cluster %d : %d   (%d, %d, %d, %d) %d \n",
								c, strokeNum, (int)CMYK[0], (int)CMYK[1], (int)CMYK[2], (int)CMYK[3], StrokeClusters[c].getClusterID());
							for (int s = 0; s < strokeNum; s++){
								StrokeClusters[c].getStroke(s).drawOnCanvas(canvas, edgeMap);
								Mat resizeCanvas;
								resize(canvas, resizeCanvas, Size(w, h));
								resizeCanvas.copyTo(largeCanvas(Rect(corner_x, corner_y, w, h)));
								imshow("Simulation", largeCanvas);
								waitKey(10);
							}
							//PAUSE
						}
						imwrite(outputFileName("image/stage", iter+1, ".jpg"), largeCanvas);
#else
						CaptureDevice >> DetectImg;
						DetectImg.copyTo(largeCanvas);

						Mat canvas = DetectImg(canvasView);
						resize(canvas, canvas, Size(imageWidth, imageHeight));

						vector<pair <Point, float>> drawPoints;
						colorDiffer(TargetImg, canvas, drawPoints, iter);
						StrokeClusters = StrokesGeneration(TargetImg, canvas, drawPoints, edgeMap, angles, iter);
						cluster_num = StrokeClusters.size();
						cluster_id = 0;
						stroke_id = 0;
						iterDone = false;

						// Draw on canvas
						printf("\n Drawing iteration : %d \n", (int)iter);
						for (int c = 0; c < StrokeClusters.size(); c++){
							int strokeNum = StrokeClusters[c].getNum();
							Vec4f CMYK = StrokeClusters[c].getColor();
							printf("  # of stroke in cluster %d : %d   (%d, %d, %d, %d) %d \n",
								c, strokeNum, (int)CMYK[0], (int)CMYK[1], (int)CMYK[2], (int)CMYK[3], StrokeClusters[c].getClusterID());
							for (int s = 0; s < strokeNum; s++){
								StrokeClusters[c].getStroke(s).drawOnCanvas(canvas, edgeMap);
								Mat resizeCanvas;
								resize(canvas, resizeCanvas, Size(w, h));
								resizeCanvas.copyTo(largeCanvas(Rect(corner_x, corner_y, w, h)));
								imshow("Simulation", largeCanvas);
								waitKey(10);
							}
							PAUSE
						}
						string filename = outputFileName("Image/Simulation", iter, ".jpg");
						imwrite(filename, largeCanvas);
						cout << "Simulation finished." << endl;
						PAUSE
				}

				

				SetCamera("draw");
				if (stroke_id < StrokeClusters[cluster_id].getNum()){
					num++;
					stroke = StrokeClusters[cluster_id].getStroke(stroke_id);
					// First of all
					if (!startF){
						DrawMode = 'm';
						startF = true;
					}
					else{
						if (DrawStroke(stroke)){
							stroke_id++;
							strokeCount++;
							if (strokeCount % 5 == 0){
								if (stroke_id < StrokeClusters[cluster_id].getNum()){
									stroke = StrokeClusters[cluster_id].getStroke(stroke_id);
									DrawMode = 'm';
								}
							}
						}
						else
							stroke_id++;
					}
				}
				else{
					cluster_id++;
					printf("\n Region %d is finished! \n", cluster_id);
					Sleep(500);
					switchColor = true;

					if (mixHistory.size() == 0){
						mixHistory.push_back(lastColor);
					}
					else{
						if (mix_id == mixHistory.size())
							mixHistory.push_back(lastColor);
					}

					if (cluster_id < cluster_num){
						stroke_id = 0;
						stroke = StrokeClusters[cluster_id].getStroke(stroke_id);
						if (StrokeClusters[cluster_id].getClusterID() != colorClass){
							printf("Class %d is finished!\n", colorClass);
							colorClass = StrokeClusters[cluster_id].getClusterID();
							printf("Next Class is %d\n", colorClass);
							DrawMode = 'c';
							PAUSE
						}

						float minDiffer = INFINITE;
						int bestIndex = 0;
						for (int i = 0; i < mixHistory.size(); i++){
							float colorDiffer = StrokeClusters[cluster_id].computeDiffer(mixHistory[i])*100. / 255.;
							if (colorDiffer < minDiffer){
								bestIndex = i;
								minDiffer = colorDiffer;
							}
						}
						if (minDiffer < ColorChangeThres){
							mix_id = bestIndex;
						}
						else{
							mix_id = mixHistory.size();
						}
						
						if (StrokeClusters[cluster_id].computeDiffer(lastColor)*100. / 255. > ColorChangeThres || strokeCount % 8 == 0){
							strokeCount = 0;
							DrawMode = 'm';
						}
					}
					else{
						printf("\n Iteration %d is finished. \n ", iter);
						iterDone = true;
						PAUSE
						DrawMode = 'c';
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
	}
	kbCmd = ' ';
}

int main(int argc, char **argv, char **envp)
{
	//mixHistory.push_back(Vec4f(0, 0, 0, 0));
	//mixHistory.push_back(Vec4f(0, 0, 0, 0));
	//mixHistory.push_back(Vec4f(0, 0, 0, 0));
	//mixHistory.push_back(Vec4f(0, 0, 0, 0));
	//mixHistory.push_back(Vec4f(0, 0, 0, 0));
	Init_IMPCard();
	Close_IMPCard();
	// Initialize Camera
	CaptureDevice.open(0);
	if (!CaptureDevice.isOpened()){
		cout << "Camera not found!";
		PAUSE
	}
	Sleep(1000);
	// Read first draw strokes
	UPDrawStrokes = readUPstroke(cluster_num, picture_id);


#if ROBOT_ON
	outputFile.open("record.txt");
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
	
	EMapConstruct(TargetImg, edgeMap, angles);

	
	// Initialize Finger Pose
	//initFinger();

	stroke = UPDrawStrokes[0].getStroke(0);
	while (true)
	{
		// Get keyboard command
		if (_kbhit()) kbCmd = _getche();
		if (kbCmd == kb_ESC) break;

		system("cls");
		DisplayLoop();
		printf("\n  Moving speed: %.2f	 Drawing Mode: %c \n ", speed, DrawMode);
		printf("\n %d / %d", num, totalnum);
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

