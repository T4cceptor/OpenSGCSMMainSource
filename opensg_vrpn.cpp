#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>

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

#include "GameModel.h"
#include "VRGPhysicsObject.h"
#include "PhysicsController.h"
#include "NodeFactory.h"
#include "Config.h"

OSG_USING_NAMESPACE

OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr;
vrpn_Tracker_Remote* tracker =  nullptr;
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;

//selfmade classes
GameModel gameModel;

int tempY;
Vec4f tempCamTo;
Pnt3f camFrom;
Pnt3f camTo;
Vec3f camUp;
int winWidth, winHeight, mouseX, mouseY;
float horizontalAngle = 0, verticalAngle = 0;
float mousespeed;
int state;
int currentPosition = 0;

Quaternion wand_orientation = Quaternion();
Vec3f wand_position = Vec3f();
Vec3f wand_direction = Vec3f();

Vec3f start_position = Vec3f(0,0,0);
Vec3f end_position = Vec3f(0,0,0);

void cleanup()
{
	delete mgr;
	delete tracker;
	delete button;
	delete analog;
}

void print_tracker();

template<typename T>
T scale_tracker2cm(const T& value)
{
	static const float scale = OSGCSM::convert_length(cfg.getUnits(), 1.f, OSGCSM::CAVEConfig::CAVEUnitCentimeters);
	return value * scale;
}

auto head_orientation = Quaternion(Vec3f(0.f, 1.f, 0.f), 3.141f);
auto head_position = Vec3f(0.f, 170.f, 200.f);	// a 1.7m Person 2m in front of the scene

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
}

auto analog_values = Vec3f();

void VRPN_CALLBACK callback_analog(void* userData, const vrpn_ANALOGCB analog)
{
	if (analog.num_channel >= 2)
		analog_values = Vec3f(analog.channel[0], 0, -analog.channel[1]);
}

void VRPN_CALLBACK callback_button(void* userData, const vrpn_BUTTONCB button)
{
	if (button.button == 0){
		if(button.state == 1)
			state = 1;
		else if(button.state == 0)
			state = 0;
	} else if (button.button == 1){
		if(button.state == 1)
			state = 2;
		else if(button.state == 0)
			state = 0;
	} else if (button.button == 2){
		if(button.state == 1){
			start_position = wand_position;
		} else if (button.state == 0){
			end_position = wand_position;
			Vec3f direction = end_position - start_position;
			std::cout << "moving hook: " << direction << std::endl;
			gameModel.moveHook(mgr->getTranslation() + direction * 10, direction * 100);
		}	
	} else if(button.button == 3){
		start_position = Vec3f(0,0,0);
		end_position = Vec3f(0,0,0);
		gameModel.moveHook(mgr->getTranslation() + Vec3f(0,-250,-1000), Vec3f(0,0,0));
		print_tracker();
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

/*----------9.1----------*/
Action::ResultE enter(Node* node){
	// if(node->getCore()->getType().isDerivedFrom(ComponentTransform::getClassType()))
	std::cout << "Enter node : " << node << std::endl;
	return Action::Continue;
}

Action::ResultE leave(Node* node, Action::ResultE result){
	std::cout << "Leaving node: " << node << "with code: " << result << std::endl;
	return result;
}


bool isGrounded(){
	//Vec3f cavePosition = mgr->getTranslation();
	//Vec3f playerPosition = cavePosition + Vec3f(head_position[0], 0, head_position[2] - 200); // head_position

	//float distance = (pltPositions::positions[currentPosition] - playerPosition).length();

	////std::cout << "cavePosition: " << cavePosition 
	////	<< " , head_position: " << head_position 
	////	<< " , playerPosition: " << playerPosition 
	////	<< " , distance: " << distance 
	////	<< std::endl;

	//if(distance > pltPositions::maxDistance * general::scale){
	//	return false;
	//}

	//return true;

	Line ray = Line(mgr->getTranslation() - Vec3f(0, 1 * general::scale, 0), Vec3f(0,-1,0));
	// IntersectAction *iAct = (IntersectActionRefPtr)IntersectAction::create();
	IntersectActionRefPtr iAct = (IntersectActionRefPtr)IntersectAction::create();
    iAct->setLine(ray);
	NodeRefPtr someNode = gameModel.getCave().getRootNode();

	//OSG::SceneGraphPrinter sgp(someNode);
	//sgp.printDownTree(std::cout);
	// traverse(someNode, enter, leave);
	iAct->apply((Node * const)someNode);

    if (iAct->didHit())
    {
		float dis = (iAct->getHitPoint().subZero() - mgr->getTranslation()).length();
		//std::cout << "distance: " << dis << std::endl;
		if(dis > general::minDistanceToFloor * general::scale ){
			return false;
		}
		return true;
    }
	return false;
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
				case '1':
						state = 1;
						std::cout << "new state: " << state << '\n';
                        break;
				case '2':
						state = 2;
						std::cout << "new state: " << state << '\n';
                        break;
				case 't':
						std::cout << "state: " << state << '\n';
						isGrounded();
                        break;
				case 'c':
						currentPosition = (currentPosition + 1) % pltPositions::size;
						mgr->setTranslation(pltPositions::positions[currentPosition] * general::scale);
						mgr->setYRotate(pltPositions::rotation[currentPosition]);
						std::cout << "moving to position: " << currentPosition << '\n';
                        break;
				case 'v':
						currentPosition = (currentPosition - 1) % pltPositions::size;
						mgr->setTranslation(pltPositions::positions[currentPosition] * general::scale);
						mgr->setYRotate(pltPositions::rotation[currentPosition]);
						std::cout << "moving to position: " << currentPosition << '\n';
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
                        std::cout << "mgr translation: " << mgr->getTranslation() << std::endl;
						std::cout << "mgr y rotation: " << mgr->getYRotate() << std::endl;
                        break;
                case 'r':
                        mgr->setTranslation(Vec3f(10,10,0));
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

Quaternion MatrixLookAt(OSG::Pnt3f from, OSG::Pnt3f at, OSG::Vec3f up){
		// TODO Performance upgrade!!!
		Vec3f view = at - from;
		view.normalize();
		Vec3f right = up % view;
		right.normalize();
		Vec3f newup = view % right;
		Vec3f objForward = Vec3f(0,1,0); 
		Vec3f objUp = Vec3f(0,1,0);
		float dot2 =  right * objForward;
		Vec3f newView = Vec3f(view[0],view[1],0);
		newView.normalize();
		float realDotXYPlane = newView * objForward;
		float realAngle = acos(realDotXYPlane);
		return Quaternion(objUp, dot2 > 0 ? -realAngle : realAngle);
		// Quaternion q1 = Quaternion(objUp, dot2 > 0 ? -realAngle : realAngle);
		/*
		float dot3 = newup * objUp;
		float angle2 = 0.0f;
		if(abs(abs(dot3) - 1.0f) > 0.0001f){
			angle2 = acos(dot3);
		}
		float dot4 = view * objUp;
		Quaternion q2 = Quaternion(Vec3f(1,0,0), dot4 < 0 ? -angle2 : angle2);
		*/
		// return q1;
}

void motion(int x, int y) {
        float deltaX = (mouseX - x);
        float deltaY = (mouseY - y);
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
	// Vec3f direction = (camTo - camFrom) * 50;
	tempCamTo.normalize();
	Vec3f movementDirection = Vec3f(tempCamTo[0],tempCamTo[1],tempCamTo[2]);
	gameModel.moveHook(
		mgr->getTranslation() + movementDirection * hook::movementOffsetScale * general::scale, 
		movementDirection * hook::movementVectorScale * general::scale
	);

	// gameModel.createNewHook(camTo, direction);
	// gameModel.createNewLight(camTo);
}

void mouse(int button, int state, int x, int y) {
	// react to mouse button presses
	//if (state) {
	//	mgr->mouseButtonRelease(button, x, y);
	//} else {
	//	mgr->mouseButtonPress(button, x, y);
	//}
	if(button == GLUT_RIGHT_BUTTON){
		if(!state){
			rightMouseButtonFunction();
		}
	}
	glutPostRedisplay();
}

void enableMouseCamera(){
	glutWarpPointer(winWidth/2, winHeight/2);
	mouseX = winWidth/2;
	mouseY = winHeight/2;
	//mousePressed = true;
	//oldTimeSinceStart = 0;
}


bool prepToStop = false;
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
		// TODO: Performance improvements
		if(state == 1){
			// gameModel.physicCtrl.calculateNewTick();
			gameModel.physicCtrl.calculateNewTickForPhysicsObject(gameModel.getHook());
			Vec3f direction = gameModel.getHook().getDirection();
			if(direction.length() > 0){
				bool didHit = gameModel.physicCtrl.collision(gameModel.getHook(), gameModel.getCave());
				if(didHit){
					// std::cout << "did Hit" << std::endl;
					prepToStop = true;
					// Vec3f newDirection = Vec3f(-direction[0] * 0.7,direction[1] * 0.9,-direction[2] * 0.7) ;
					// gameModel.getHook().setDirection(newDirection * 0.7);
				} else if(prepToStop){
					prepToStop = false;
					// gameModel.getHook().setDirection(0,0,0);
					// Vec3f newDirection = Vec3f(-direction[0] * 0.7,direction[1] * 0.9,-direction[2] * 0.7) ;
					gameModel.getHook().setDirection(gameModel.physicCtrl.getReflectionVector() * 0.8);
				}
			}
			// TODO:
			// if(didHitPlattform(gameModel.getHook())){
			//		changeGameState();
			// // evtl. in anderen GameState verschieben
			//		animateHook(); // TODO!
			//		animateRope(); // TODO!
			//		when finished animation:
			//		moveToNextPlattform();
			//		changeGameState();
			// }

			if(!isGrounded()){
				mgr->setTranslation(mgr->getTranslation() - general::upVector * general::scale);
			}
		}

		/*
		if(state == 1){
			mgr->setTranslation(mgr->getTranslation() + wand_direction * 250);
		} else if (state == 2){
			mgr->setTranslation(mgr->getTranslation() - wand_direction * 250);
		}
		*/
		
		// mgr->setTranslation(mgr->getTranslation() + speed * analog_values);
		check_tracker();
		const auto speed = 1.f;
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

	camFrom = * new Pnt3f(0,0,0);
	camTo = * new Pnt3f(1,0,0);
	camUp = * new Vec3f(0,0,1);
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
		std::cout << "config test: " << config::test << std::endl;
		std::cout << "config test2: " << physics::speed << std::endl;
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

		mousespeed = 0.0005;
		tempY = 0;
		state = 0;

		NodeFactory nodeFa = * new NodeFactory();
		gameModel = * new GameModel();
		gameModel.initGameModel(nodeFa);
		gameModel.createScenegraph();
		NodeRecPtr root = gameModel.getScenegraphRoot().getRootNode();

		commitChanges();

		mgr = new OSGCSM::CAVESceneManager(&cfg);
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

	glutMainLoop();
}
