#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>
#include <time.h>

#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OSGIntersectAction.h>
#include <OSGSimpleGeometry.h>
#include <OpenSG/OSGFieldContainerUtils.h>
#include <OSGSceneGraphUtils.h>
#include <OSGSimpleAttachment.h>
#include <OSGAttachment.h>

#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>
#include <OSGCSM/appctrl.h>

#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>

#include "GameController.h"
#include "Config.h"

OSG_USING_NAMESPACE

// CAVE variables
OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr;
vrpn_Tracker_Remote* tracker =  nullptr;
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;

void print_tracker();
void updateCurrentDirection(Vec3f newPosition);

auto head_orientation = Quaternion(Vec3f(0.f, 1.f, 0.f), 3.141f);
auto head_position = Vec3f(0.f, 170.f, 200.f);	// a 1.7m Person 2m in front of the scene

auto analog_values = Vec3f();

// controls every aspect of the game
GameController gCtrl; 

// current look vector of the player / cave
Vec4f tempCamTo;

// mouse input variables
bool leftMouseDown = false;
bool rightMouseDown = false;
int winWidth, winHeight, mouseX, mouseY;
float mouseDistance = 0;

// wand input variables
int buttonPressed = -1;
Quaternion wand_orientation = Quaternion();
Vec3f wand_position = Vec3f();
Vec3f wand_direction = Vec3f();
Vec3f start_position = Vec3f(0,0,0);
Vec3f end_position = Vec3f(0,0,0);
Vec3f lastPosition;
Vec3f currentDirection;

float throwScale = 1.0f;


void cleanup() {
	delete mgr;
	delete tracker;
	delete button;
	delete analog;
}

template<typename T>
T scale_tracker2cm(const T& value)
{
	static const float scale = OSGCSM::convert_length(cfg.getUnits(), 1.f, OSGCSM::CAVEConfig::CAVEUnitCentimeters);
	return value * scale;
}

void VRPN_CALLBACK callback_head_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	head_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	head_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

void VRPN_CALLBACK callback_wand_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	wand_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	wand_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
	wand_direction =  Vec3f(wand_orientation[1], wand_orientation[2], wand_orientation[3]);

	if(buttonPressed == 2){
		updateCurrentDirection(wand_position);
	}
}

void VRPN_CALLBACK callback_analog(void* userData, const vrpn_ANALOGCB analog)
{
	if ( analog.num_channel >= 2 ){
		analog_values = Vec3f(analog.channel[0], 0, -analog.channel[1]);
		std::cout << "analog.channel[0]: " << analog.channel[0] << " ,-analog.channel[1]: " << -analog.channel[1] << "\n";
		
		if(analog.channel[0] >= 0.8 || analog.channel[0] <= -0.8){
		  float newRotation = mgr->getYRotate() - analog.channel[0] / 20;
		  mgr->setYRotate(newRotation);
		}
		if(analog.channel[1] >= 0.8 || analog.channel[1] <= -0.8){
		  throwScale += analog.channel[1] / 10;
		  std::cout << "throw scale: " << throwScale << "\n";
		}
		
	}
}

void VRPN_CALLBACK callback_button(void* userData, const vrpn_BUTTONCB button)
{
	if(button.state == 0)
		buttonPressed = -1;
	else if(button.state == 1)
		buttonPressed = button.button;

	if (button.button == 0){ 
		if(button.state == 1){
			gCtrl.resetGameState(1);
		}
	} else if (button.button == 2){
		if(button.state == 1){
			currentDirection = Vec3f(0,0,0);
			start_position = wand_position;
		} else if (button.state == 0){
			if(gCtrl.getGameState() == 1){
				float rotation = mgr->getYRotate();
				Vec3f direction = currentDirection;
				direction.normalize();
				Vec3f newDirection = Matrix(
					Vec3f(cos(rotation), 	0, 	-sin(rotation)),
					Vec3f(0, 		1, 	0),
					Vec3f(sin(rotation),	0,	cos(rotation))
				) * direction;
				newDirection.normalize();
				gCtrl.moveHook(-newDirection, currentDirection.length() * throwScale);
			} else if (gCtrl.getGameState() == 2){
				// TODO
			}else if(gCtrl.getGameState() == 3){
				// TODO
			}
		}	
	} else if(button.button == 3){
	    if(button.state == 1){
		gCtrl.jumpToNextPlattform();
	    }
	}
}

// TODO, evtl herzeigen
void updateCurrentDirection(Vec3f newPosition){
	if(lastPosition != newPosition){
		currentDirection = currentDirection * 0.5 + (newPosition - lastPosition);	
	}
	lastPosition = newPosition;

	if(gCtrl.getGameState() == 1){
		// TODO
	} else if(gCtrl.getGameState() == 3){
		gCtrl.moveTowardsPlattform(currentDirection);
		
	}
}

void InitTracker(OSGCSM::CAVEConfig &cfg)
{
	try
	{
		const char* const vrpn_name = "DTrack@localhost";
		tracker = new vrpn_Tracker_Remote(vrpn_name);
		tracker->shutup = true;
		tracker->register_change_handler(NULL, callback_head_tracker, cfg.getSensorIDHead());
		tracker->register_change_handler(NULL, callback_wand_tracker, cfg.getSensorIDController());
		button = new vrpn_Button_Remote(vrpn_name);
		button->shutup = true;
		button->register_change_handler(nullptr, callback_button);
		analog = new vrpn_Analog_Remote(vrpn_name);
		analog->shutup = true;
		analog->register_change_handler(NULL, callback_analog);
	}
	catch(const std::exception& e) 
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return;
	}
}

void check_tracker()
{
	tracker->mainloop();
	button->mainloop();
	analog->mainloop();
}

void print_tracker()
{
	std::cout << "Head position: " << head_position << " orientation: " << head_orientation << '\n';
	std::cout << "Wand position: " << wand_position << " orientation: " << wand_orientation << '\n';
	std::cout << "Analog: " << analog_values << '\n';
	std::cout << "direction: " << wand_direction << '\n';
}

void keyboard(unsigned char k, int x, int y)
{
	tempCamTo.normalize();
	Vec3f movementDirection = Vec3f(tempCamTo[0],tempCamTo[1],tempCamTo[2]);
	Vec3f rightDirection = (movementDirection % general::upVector);
	float multiplier = general::movementFactor * general::scale;
	Real32 ed;
	switch(k)
	{
	case 27:
		cleanup();
		exit(EXIT_SUCCESS);
		break;
	case 'q':
		gCtrl.resetGame();
		break;
	case 'r':
		gCtrl.resetGameState(gCtrl.getGameState());
		break;
	case '1':
		gCtrl.setGameState(0);
		std::cout << "new state: " << gCtrl.getGameState() << '\n';
		break;
	case '2':
		gCtrl.setGameState(1);
		std::cout << "new state: " << gCtrl.getGameState() << '\n';
		break;
	case 't':
		break;
	case 'c':
		gCtrl.jumpToNextPlattform();
		break;
	case 'v':
		gCtrl.jumpToPreviousPlattform();
		break;
	case 'e':
		ed = mgr->getEyeSeparation() * .9f;
		std::cout << "Eye distance: " << ed << '\n';
		mgr->setEyeSeparation(ed);
		break;
	case 'E':
		ed = mgr->getEyeSeparation() * 1.1f;
		std::cout << "Eye distance: " << ed << '\n';
		mgr->setEyeSeparation(ed);
		break;
	case 'h':
		cfg.setFollowHead(!cfg.getFollowHead());
		std::cout << "following head: " << std::boolalpha << cfg.getFollowHead() << '\n';
		break;
	case 'i':
		print_tracker();
		break;
	case 'b':
		break;
	case 'w':
		mgr->setTranslation(mgr->getTranslation() - movementDirection * multiplier);
		// mgr->
		break;
	case 'a':
		mgr->setTranslation(mgr->getTranslation() + rightDirection * multiplier);
		break;
	case 's':
		mgr->setTranslation(mgr->getTranslation() + movementDirection * multiplier);
		break;
	case 'd':
		mgr->setTranslation(mgr->getTranslation() - rightDirection * multiplier);
		break;
	case 'f':
		std::cout << "mgr position: " << mgr->getTranslation() << std::endl;
		std::cout << "mgr y rotation: " << mgr->getYRotate() << std::endl;
		std::cout << "hook position: " << gCtrl.getModel()->getHook().getPosition() << std::endl;
		std::cout << "hook look at: " << gCtrl.getModel()->getHook().getLookAt() << std::endl;
		break;
	case 'y':
		mgr->setYRotate(mgr->getYRotate() + 0.1f);
		tempCamTo = Vec4f(sin(mgr->getYRotate()),0,cos(mgr->getYRotate()),0);
		break;
	case 'x':
		mgr->setYRotate(mgr->getYRotate() - 0.1f);
		tempCamTo = Vec4f(sin(mgr->getYRotate()),0,cos(mgr->getYRotate()),0);
		break;
	default:
		std::cout << "Key '" << k << "' ignored\n";
	}
}

void motion(int x, int y) {
	float deltaX = (mouseX - x);
	float deltaY = (mouseY - y);
	if(!leftMouseDown){
		mouseDistance += deltaY;
		return;
	}
	if(deltaX != 0){
		mgr->setYRotate(mgr->getYRotate() + 0.01 * deltaX);
		tempCamTo = Vec4f(sin(mgr->getYRotate()),0,cos(mgr->getYRotate()),0);
		mouseX = x;
		deltaX = x;
	}
	glutPostRedisplay();
	if(x <= 35 || x > winWidth - 35 || y <= 35 || y > winHeight - 35){
		glutWarpPointer(winWidth/2, winHeight/2);
		mouseX = winWidth/2;
		mouseY = winHeight/2;
	}

}

void rightMouseButtonFunction(){
	if(gCtrl.getGameState() == 1){
		float rotation = mgr->getYRotate();
		Vec3f tempVec = Matrix(
					Vec3f(cos(rotation), 	0, 	-sin(rotation)),
					Vec3f(0, 		1, 	0),
					Vec3f(sin(rotation),	0,	cos(rotation))
				) * Vec3f(0, 0, 1);
		tempVec.normalize();
		Vec3f movementDirection = Vec3f(tempVec[0],tempVec[1],tempVec[2]);
		gCtrl.moveHook(movementDirection, abs(mouseDistance) / 100);
	}
}

void mouse(int button, int state, int x, int y) {
	if(button == GLUT_RIGHT_BUTTON){
		if(!state){
			mouseDistance = 0;
			rightMouseDown = true;
			gCtrl.rightMouseDown = true;
		} else {
			rightMouseButtonFunction();
			rightMouseDown = false;
			gCtrl.rightMouseDown = false;
		}
	}
	if(button == GLUT_LEFT_BUTTON){
		if(state){
			leftMouseDown = false;
			gCtrl.leftMouseDown = false;

		} else {
			leftMouseDown = true;
			gCtrl.leftMouseDown = true;
			
			Vec3f m = Vec3f(tempCamTo[0], tempCamTo[1], tempCamTo[2]) * 20;
			gCtrl.moveTowardsPlattform(m);
		}
	}
	glutPostRedisplay();
}

void setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB  |GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("OpenSG CSMDemo with VRPN API");
	glutDisplayFunc([]()
	{
		// black navigation window
		glClear(GL_COLOR_BUFFER_BIT);
		glutSwapBuffers();
	});
	glutReshapeFunc([](int w, int h)
	{
		mgr->resize(w, h);
		winWidth = w;
		winHeight = h;
		mouseX = winWidth/2;
		mouseY = winHeight/2;
		glutPostRedisplay();
	});
	glutKeyboardFunc(keyboard);
	glutIdleFunc([]()
	{
		// GameLoop gets called, this should be implemented in a seperated thread (TODO)
		gCtrl.callGameLoop(); 

		check_tracker();
		const auto speed = 1.f; // needed - ?
		mgr->setUserTransform(head_position, head_orientation); // dont touch
		commitChanges();
		mgr->redraw();

		// the changelist should be cleared - else things could be copied multiple times
		OSG::Thread::getCurrentChangeList()->clear();
	});
	glutMotionFunc(motion);
	glutMouseFunc(mouse);
	// enableMouseCamera();
	// glutPassiveMotionFunc(motion);

	tempCamTo = * new Vec4f(0,0,-1,0);
}


int main(int argc, char **argv)
{
#if WIN32
	OSG::preloadSharedObject("OSGFileIO");
	OSG::preloadSharedObject("OSGImageFileIO");
#endif
	try
	{
		bool cfgIsSet = false;
		// ChangeList needs to be set for OpenSG 1.4
		ChangeList::setReadWriteDefault();
		osgInit(argc,argv);
		// evaluate intial params
		for(int a=1 ; a<argc ; ++a)
		{
			std::cout << "argc: " << a << std::endl;
			if( argv[a][0] == '-' )
			{
				if ( strcmp(argv[a],"-f") == 0 ) 
				{
					char* cfgFile = argv[a][2] ? &argv[a][2] : &argv[++a][0];
					if (!cfg.loadFile(cfgFile)) 
					{
						std::cout << "ERROR: could not load config file '" << cfgFile << "'\n";
						return EXIT_FAILURE;
					}
					cfgIsSet = true;
				}
			} else {
				std::cout << "Loading scene file '" << argv[a] << "'\n";
				// scene = SceneFileHandler::the()->read(argv[a], NULL);
			}
		}

		// load the CAVE setup config file if it was not loaded already:
		if (!cfgIsSet) 
		{
			const char* const default_config_filename = "config/mono.csm";
			if (!cfg.loadFile(default_config_filename)) 
			{
				std::cout << "ERROR: could not load default config file '" << default_config_filename << "'\n";
				return EXIT_FAILURE;
			}
		}
		cfg.printConfig();

		// start servers for video rendering
		if ( startServers(cfg) < 0 ) 
		{
			std::cout << "ERROR: Failed to start servers\n";
			return EXIT_FAILURE;
		}

		setupGLUT(&argc, argv);
		InitTracker(cfg);
		MultiDisplayWindowRefPtr mwin = createAppWindow(cfg, cfg.getBroadcastaddress());

		currentDirection = Vec3f(0,0,0);
		lastPosition = Vec3f(0,0,0);

		commitChanges();
		mgr = new OSGCSM::CAVESceneManager(&cfg);

		gCtrl = * new GameController();
		// GameController gets initialised with the CAVESceneManager
		gCtrl.init(mgr);
		// the scenegraph is created and returned
		NodeRecPtr root = gCtrl.setupScenegraph();
		
		mgr->setWindow(mwin );
		mgr->setRoot(root);
		mgr->showAll();
		mgr->getWindow()->init();
		mgr->turnWandOff();
		mgr->setHeadlight(false);
		mgr->setYRotate(1.0f);

	}
	catch(const std::exception& e)
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return EXIT_FAILURE;
	}

	// game is started, now input is possible
	gCtrl.startGame();
	glutMainLoop();
}