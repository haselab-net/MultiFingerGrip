#ifndef TWO_FINGER_H
#define TWO_FINGER_H

//FLEXI FORCE CALIBRATION PARAMETERS
#define SATURATION_VOLTAGE 0.624
#define NO_LOAD_VOLTAGE    1.763
#define NO_LOAD_NEWTON     0
#define SATURATION_NEWTON  4.4

//force display parameters
#define VIBBUF_LEN 100

//defines the angle detection range (in +,- degrees)
#define ANGLE_RANGE 20

//number of jenga objects included in the sprfile 
#define JENGA_NUMBER 3

#include <Springhead.h>
#include <Framework\SprFWApp.h>
#include <HumanInterface\SprHIDRUsb.h>

#include <windows.h>  //This library should be at the bottom
#include <iomanip>



using namespace Spr;
using namespace std;

class TwoFinger : public FWApp{
public: //Local Properties
	enum HumanInterface{
		SPIDAR,
		XBOX,
		FALCON,
	} humanInterface;
	
	HIBaseIf* spg;
	HISpidarGIf* spidar;
	PHSceneIf* phscene;
	FWSceneIf* fwscene;
	PHHapticEngineIf* hapscene;
	DVAdIf* flexiforce;
	GRLightDesc ld;    //lights used to draw letters and the force grap in the screen

	UTRef<UTTimerIf> ptimer;
	int pTimerID;
	string fileName;
	int sceneNumber;
	float pdt;
	int nsolids;
	double posScale;  //2.5 orignal value with 20x30 floor scene (Virgilio original)
	int grabKey;	//	'1' to '9'

	TwoFinger();
	void InitHapticInterface();
	void Init(int argc, char* argv[]);
	void BuildScene();
	void InitCameraView();
	virtual void TimerFunc(int id);
	virtual void Keyboard(int key, int x, int y);
	virtual void Display(); //virtual fucntion used to display graphics
	virtual void AtExit();  //used to close the debug file
	void TwoFingerStep(Vec3f *spidarForce, Vec3f *spidarTorque);  //encapsulates the twofinger calculations, returns the force to be displayed in Spidar

	//phsolid objects assigned during buildscene()
	PHSolidIf* fPointer1;
	PHSolidIf* fPointer2;
	PHSolidIf *fPhone;
	PHSolidIf *fHammer;
	PHSolidIf *fJenga1;
	//PHSolidIf *fJenga2;
	//PHSolidIf *fJenga3;
	PHSolidIf *fAluminio;

	//Used for the two finger manipulation method
	Posed defaultCenterPose;
	Posed defaultPose1;
	Posed defaultPose2;
	double maxReach;
	double grabForce;

	double vibBuffer[1000];
	int count;  //used to fill the buffer of the force graph
	int delay;  //used to fill the buffer of the force graph

	void calibrate(); //calibrates SPIDAR and orientates the pointers
	void displayGraph(GRRenderIf* render);  //draws the force graphic on the right of the screen

	//new version of the demonstration
	//ofstream myfile;   //file to DEBUG
	void resetObjects();  //resets the objects position
	bool displayGraphFlag;  //flag to display the force sensor graph
};

#endif