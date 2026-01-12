#include "MultiFinger.h"
#include <windows.h>
#include <conio.h>
#include <Springhead.h>
#include <HumanInterface/SprHIDRUsb.h>
#include <HumanInterface/SprHIKeyMouse.h>
#include <Foundation/SprUTQPTimer.h>
#include <iomanip>
#include "Logger.hpp"


#define USER_NUM 15

//Constructor 
MultiFinger::MultiFinger(){
	humanInterface = SPIDAR;

	pdt = 0.001f;
	posScale = 10.0;  //2.5 orignal value with 20x30 floor scene (Virgilio original) (for peta pointer)  10.0 

	displayGraphFlag = false;
	logger = new Logger("Exp" + std::to_string(USER_NUM));
	message = "MultiFinger Grip using LuGre Friction Model.";
	increaseMassState = IDLE;
	trialNumber = 0;
	offset = 0.5f;
	inContact = false;
}

//main function of the class
void MultiFinger::Init(int argc, char* argv[]){
	FWApp::Init(argc, argv);

	InitCameraView();

	BuildScene();
	sceneNumber = GetSdk()->NScene() - 1;
	GetWin(0)->SetScene(GetSdk()->GetScene(sceneNumber));
	InitHapticInterface();
		
	ptimer = CreateTimer(UTTimerIf::MULTIMEDIA);
	pTimerID = ptimer->GetID();
	//DSTR << "timer id xxx: " << pTimerID << std::endl;
	ptimer->SetResolution(1);
	ptimer->SetInterval(1);
	ptimer->Start();

	count = 0;
	delay = 0;
}


void MultiFinger::BuildScene(){
	int i = 0;

	i = GetSdk()->NScene() - 1;
	phscene = GetSdk()->GetScene(i)->GetPHScene();
	phscene->SetTimeStep(pdt);
	//phscene->SetNumIteration(50);

	fwscene = GetSdk()->GetScene(i);
	//fwscene->EnableRenderAxis(false, false, true);
	//fwscene->EnableRenderForce(false, true);
	//fwscene->EnableRenderContact(false);
	GetSdk()->SetDebugMode(true);
	
	phscene->SetGravity(Vec3d(0.0, -9.8, 0.0));

	// Floor
	PHSolidIf* floor = phscene->CreateSolid();
	CDBoxDesc bd;
	bd.boxsize = Vec3d(1.0, 0.025, 0.50);
	CDShapeIf* sh = GetSdk()->GetPHSdk()->CreateShape(bd);
	PHMaterial mat;
	mat.frictionModel = FrictionModel::COULOMB;
	mat.mu = 0.8f;	mat.mu0 = 0.6f;
	sh->SetMaterial(mat);
	floor->AddShape(sh);
	floor->SetFramePosition(Vec3d(0.0, -0.025, 0.0));
	floor->SetDynamical(false);

	// Target Object
	target = phscene->CreateSolid();
	bd.boxsize = Vec3d(0.06, 0.12, 0.06);
	sh = GetSdk()->GetPHSdk()->CreateShape(bd);
	sh->SetMaterial(mat);
	target->AddShape(sh);
	target->SetFramePosition(Vec3d(0.0, 0.07, 0.0));
	const double I = 1.0e6;
	target->SetInertia(Matrix3d(
		I, 0.0, 0.0, 
		0.0, I, 0.0,
		0.0, 0.0, I));
	//target->CompInertia();

	// Push Object
	CDCapsuleDesc capDesc;
	capDesc.radius = 0.01;
	capDesc.length = 0.1;
	CDShapeIf *cap = GetSdk()->GetPHSdk()->CreateShape(capDesc)->Cast();
	cap->SetDensity(0.0f);
	pushObject = phscene->CreateSolid();
	pushObject->AddShape(cap);
	pushObject->SetFramePosition(Vec3d(0.0, 0.3, 0.0));
	pushObject->SetOrientation(Quaterniond().Rot(M_PI / 2.0, 'x'));
	pushObject->SetDynamical(false);
	fwscene->SetSolidMaterial(GRRenderIf::TMaterialSample::DODGERBLUE, pushObject);

	// Finger Grip
	grip.Build(fwscene);
	maxReach = 0.05;
	this->nsolids = phscene->NSolids();

	fwscene->GetPHScene()->SetContactMode(pushObject, PHSceneDesc::MODE_NONE);

	SetNext(true);
}

//Inits SPIDAR and calibrates the pointer position
void MultiFinger::InitHapticInterface() {
	HISdkIf* hiSdk = GetSdk()->GetHISdk();

	bool bFoundCy = false;
	if (humanInterface == SPIDAR) {
		// x64
		DRCyUsb20Sh4Desc cyDesc;
		for (int i = 0; i < 10; ++i) {
			cyDesc.channel = i;
			DRCyUsb20Sh4If* cy = hiSdk->AddRealDevice(DRCyUsb20Sh4If::GetIfInfoStatic(), &cyDesc)->Cast();
			if (cy && cy->NChildObject()) {
				bFoundCy = true;
			}
			else {
				hiSdk->DelChildObject(cy);
			}
		}
		DRUARTMotorDriverDesc uartDesc;
		uartMotorDriver = hiSdk->AddRealDevice(DRUARTMotorDriverIf::GetIfInfoStatic(), &uartDesc)->Cast();
		hiSdk->AddRealDevice(DRKeyMouseWin32If::GetIfInfoStatic());
		hiSdk->Print(DSTR);
		hiSdk->Print(std::cout);

		spidar = hiSdk->CreateHumanInterface(HISpidarGIf::GetIfInfoStatic())->Cast();
		if (bFoundCy) {
			spidar->Init(&HISpidarGDesc("SpidarG6X3R")); //Original SPIDARG6
			flexiforce = hiSdk->RentVirtualDevice(DVAdIf::GetIfInfoStatic(), "", 3)->Cast(); // 圧力センサ
			std::cout << "Init SpidarG6X3R" << std::endl;
		}
		else {
			spidar->Init(&HISpidarGDesc("SpidarG6X4R"));	//	low price X SPIDAR
			std::cout << "Init SpidarG6X4R" << std::endl;
			DSTR << "Init SpidarG6X4R" << std::endl;
		}
		spidar->Calibration();
		spidar = spidar->Cast();
	}
	else if (humanInterface == XBOX) {
		spidar = hiSdk->CreateHumanInterface(HIXbox360ControllerIf::GetIfInfoStatic())->Cast();
	}
	else if (humanInterface == FALCON) {
		spidar = hiSdk->CreateHumanInterface(HINovintFalconIf::GetIfInfoStatic())->Cast();
		spidar->Init(NULL);
	}
	else {

	}
	/*
	//The port is 3 because the sensor is connected to the port 3 on the Spidar's AD Converter
	if (bFoundCy) {
		flexiforce = hiSdk->RentVirtualDevice(DVAdIf::GetIfInfoStatic(), "", 3)->Cast();
	}
	else {
		flexiforce = hiSdk->RentVirtualDevice(DVAdIf::GetIfInfoStatic(), "", 2)->Cast();
	}
	
	if (flexiforce && 1700 < flexiforce->Digit() && flexiforce->Digit() < 1900) {
		flexiforce = NULL;
	}
	*/
}
//void MultiFinger::MultiFingerStep(Vec3f* spidarForce)
//{
//}

void MultiFinger::InitCameraView(){

	Vec3d pos = Vec3d(0, 0.08, 0.4);
	GetCurrentWin()->GetTrackball()->SetPosition(pos);
	Affinef af;
	af.Pos() = pos;
	Vec3d target = Vec3d(0.0, 0.07, 0);	 //focused on the tochdown zone
	GetCurrentWin()->GetTrackball()->SetTarget(target);	// カメラ初期位置の設定
}


//Calibrates the position of the grip and both pointers
void MultiFinger::calibrate() {	
	spidar->Calibration();
	offset = flexiforce->Voltage();
	DSTR << "Calibration done. Offset: " << offset << std::endl;
}

//This multimedia thread handles the haptic (6DOF virtual coupling pointers) and physics simulation (Springhead)
void MultiFinger::TimerFunc(int id){
	//DSTR << "timers id: " << pTimerID << std::endl;
	if (pTimerID == id){
		// Count "Cycle Per Second"
		if (1){
			static int cycle = 0;
			static DWORD lastCounted = timeGetTime();
			DWORD now = timeGetTime();
			if (now - lastCounted > 1000) {
			float cps = (float)(cycle) / (float)(now - lastCounted) * 1000.0f;
			std::cout << "cps:" << cps << std::endl;
			if(flexiforce)
				std::cout << "flexiforce:"  << flexiforce->Voltage() << std::endl;
			lastCounted = now;
			cycle = 0;
			int a = 0;
			}
			cycle++;
		}
		UTAutoLock LOCK(displayLock);
		inContact = false;

		phscene->Step();  //springhead physics step
		double a_vib = 0.0f;
		Posed pose = spidar->GetPose();
		pose.Pos() = pose.Pos()*4;
		pose.PosY() += 0.07;
		// Disable XZ movement and rotation
		pose.PosX() = 0.0f;
		pose.PosZ() = 0.0f;
		pose.Ori() = Quaterniond(1, 0, 0, 0);

		static int c = 0;
		c++;
		double flexiforceValue = 0.0f;
		static double flexiforce_p = 0.0f;
		bool properGraspForce = false;
		bool properHeight = target->GetFramePosition().y > 0.12;
		if (flexiforce) {
			// bad calibration! m = -1.5716   b = 2.7717  //  -2.4914    4.6105
			float volts = flexiforce->Voltage();
			const double a = 0.05;
			flexiforceValue = a * (0.4*(volts - offset)) + (1.0 - a) * flexiforce_p;
			flexiforce_p = flexiforceValue;
			properGraspForce = IsGraspForceProper(flexiforceValue);
			if(properGraspForce || increaseMassState >= INCREASE)
				fwscene->SetSolidMaterial(GRRenderIf::TMaterialSample::WHITE, target);
			else
				fwscene->SetSolidMaterial(GRRenderIf::TMaterialSample::RED, target);
		}
		grip.Step(pose, phscene->GetTimeStep());	//	this will be actual code.

		Vec3d totalForce, totalTorque;
		double vib = 0.0f;
		PHSolidIf* tool = grip.fingers[0].tool;
		//cout << "soAluminioLight:" << target->GetMass() << std::endl;
		bool bswap = false;
		PHSolidPairForLCPIf* sp = phscene->GetSolidPair(tool, target, bswap);
		PHShapePairForLCPIf* p = sp->GetShapePair(0, 0);
		PHContactPointIf* cp = nullptr;
		double dT = 0.0f;
		double t = phscene->GetCount() * pdt;

		// Check the state of contacts between tool and target. 
		// Record friction state and generate vibration force accordingly.
		double grabForce = 0.0f;
		bool isGrasping = false;
		static bool isGraspingPrev = false;
		static double contactStartTime = 0.0f;
		for (int i = 0; i < phscene->NContacts(); i++) {
			cp = phscene->GetContact(i);
			if ((cp->GetSocketSolid() == tool || cp->GetPlugSolid() == tool) 
				&&(cp->GetSocketSolid() == target || cp->GetPlugSolid() == target)){
				inContact = true;
				Vec3d cf, ct;
				cp->GetConstraintForce(cf, ct);
				grabForce = cf[0]; // Normal force

				isGrasping = properGraspForce && properHeight;

				static bool prev_is_static = false;
				Vec3d cv, cw;
				cp->GetRelativeVelocity(cv, cw);
				bool is_static = cv.norm() <= 5.0e-2;//cp->IsStaticFriction();
				if (!is_static && prev_is_static) {
					// Transition from static to dynamic friction
					stickSlipTime.push_back(t);
					std::cout << "stick-slip at t=" << t << std::endl;
				}
				prev_is_static = is_static;
				// Vibration
				if (logger->condition.friction_model == 0) {
					// Coulomb
					const double A1 = 15.0f;
					const double A2 = 8.0f;
					const double fa1 = 15.0f;
					const double fa2 = 150.0f;

					Vec3d v, w;
					cp->GetRelativeVelocity(v, w); 
					vib += A1 * sqrt(grabForce>0.5 ? 0.5 : grabForce) * v.norm() * sin(2.0f * M_PI * fa1 * t);
				}
				else {
					// LuGre
					double T = cp->GetLuGreT();
					Vec3d slip = cp->GetLuGreVS();
					static double T_p = 0.0f;

					dT = (T - T_p) / pdt;
					if (dT > 0.0f) {
						dT = 0.0f;
					}
					const double fa1 = 30.0f;
					const double A1 = 1000.0f;
					double s = slip.norm();
					//double slipd = min(A2 * slip.norm(), 3.0f);
					vib = min(A1* s, 1.0f)* sin(2.0f * M_PI * fa1 * t);// +A2 * slipd * sin(2.0f * M_PI * fa2 * t);
					//std::cout << dT << "," << slipd << std::endl;
					T_p = T;
				}
				const double decay = 200.0f;
				const double fa2 = 150.0f;
				const double A2 = 8.0f;
				for (float t1 : stickSlipTime) {
					if (phscene->GetCount() * pdt - t1 <= 0.2) {
						vib += A2 * sqrt(grabForce > 0.5 ? 0.5 : grabForce) * exp(-(t - t1) * decay) * sin(2.0f * M_PI * fa2 * (t - t1));
					}
					else {
						// Remove old stick-slip events
						stickSlipTime.erase(stickSlipTime.begin());
					}

				}

				// Logging
				Logger::LogData data;
				data.t = phscene->GetCount();
				data.load_pos[0] = target->GetPose().PosX();
				data.load_pos[1] = target->GetPose().PosY();
				data.load_pos[2] = target->GetPose().PosZ();
				data.pointer_pos[0] = grip.pose.PosX();
				data.pointer_pos[1] = grip.pose.PosY();
				data.pointer_pos[2] = grip.pose.PosZ();
				data.grip_force = grabForce;
				data.lugre_T = cp->GetLuGreT();
				data.lugre_v[0] = cp->GetLuGreV()[0];
				data.lugre_v[1] = cp->GetLuGreV()[1];
				data.lugre_dz[0] = cp->GetLuGreDZ()[0];
				data.lugre_dz[1] = cp->GetLuGreDZ()[1];
				data.lugre_z[0] = cp->GetLuGreZ()[0];
				data.lugre_z[1] = cp->GetLuGreZ()[1];
				data.friction_force = Vec2d(cf[1], cf[2]).norm();
				data.vibration_force = vib;
				data.is_static_friction = cp->IsStaticFriction();
				//data.mass = target->GetMass();
				logger->data = data;
				logger->saveSample();

			}
		}
		for (Finger& finger : grip.fingers) {

			finger.AddForce(flexiforceValue);	//	This must be actual force sensor values. For debug purpose only first two pointers get force.
			Vec6d couplingForce = finger.spring->GetMotorForce();
			//DSTR << "c" << finger.GetIndex() << " f=" << couplingForce << std::endl;
			//	finger.AddForce(couplingForce[0]);
			Posed socketPose;
			finger.spring->GetSocketPose(socketPose);
			Quaterniond ori = grip.gripDevice->GetOrientation() * socketPose.Ori();
			//	all in global coordinates:
			Vec3d p = grip.gripDevice->GetOrientation() * (finger.position + finger.length * finger.direction);
			Vec3d f = ori * couplingForce.sub_vector(0, Vec3d());
			Vec3d t = ori * couplingForce.sub_vector(3, Vec3d());
			totalForce += f;
			//totalTorque += t + (p % f);
			totalTorque += (p % f);
		}

		double contactDuration = -1.0f;
		if (increaseMassState == RANDOM_WAIT && !inContact) {
			increaseMassState = IDLE;
		}
		if (!isGraspingPrev && isGrasping  && increaseMassState < RANDOM_WAIT ) {
			contactStartTime = t;
			//std::cout << "contact start time: " << contactStartTime << std::endl;
		}	
		else if ( properGraspForce && properHeight && isGrasping || increaseMassState >= RANDOM_WAIT ) {
			// Stable grasp or already started increasing, proceed to increase mass
			contactDuration = t - contactStartTime;
			//std::cout << "contact duration: " << contactDuration << std::endl;
		}
		increaseMassState = IncreaseMass(contactDuration);
		isGraspingPrev = isGrasping;

		double fs = 0.3f, ts = 0.5;
		if(!bForceFeedback)
			totalForce = Vec3d::Zero();
		if (bVibrationFeedback)
			totalForce.y += vib;
		spidar->SetForce(-fs * totalForce, Vec3f());	

		spidar->Update(pdt);  //updates the forces displayed in SPIDAR
		//MultiFingerStep(&spidarForce);  //This function computes the lineal and rotational couplings value
		//spidar->SetForce(-spidarForce);  //This function set the force 
		
		if (!properGraspForce && increaseMassState <= WAIT) {
			message = "Grasp force too STRONG. Please Loosen Grip.";
		}
		else if (!properHeight && increaseMassState == WAIT) {
			message = "Object too LOW. Please Raise Up.";
		}
		else if (increaseMassState == IDLE) {
			message = "Please Lift Up the Object.";
		}
		else if (increaseMassState == WAIT) {
			message = "Please Hold STEADY.";
		}
		else if (increaseMassState == INCREASE || increaseMassState == RANDOM_WAIT) {
			message = "The Blue Cylinder will push the BOX. Please DON'T Drop it.";
		}
		else if (increaseMassState == FINISH) {
			message = "Success. Please Press Enter Key.";
		}
		PostRedisplay();

		// Debug log
		if (c % 10 == 0) {
			DSTR << "grabforce: " << grabForce << ", isProper: " << properGraspForce <<
				", contactDuration: " << contactDuration <<  std::endl;

		}
	}
	else {
		return;
	}
}

//catches keyboard events
void MultiFinger::Keyboard(int key, int x, int y){
	if (ptimer){
		while (!ptimer->Stop());
	}
	int spKey = key - 0x100;
	Posed pose = grip.gripDevice->GetPose();
	const double d = 0.01;
	bool stopTimer = false;
	switch (key) {
	case 13:
		// Enter key
		if (increaseMassState == FINISH) {
			SetNext(false);
		}
		break;
	case 'p':
		SetNext(true);
		break;
	case 27:
	case 'q':
		logger->close();
		exit(0);
		break;
	case 'g':
		GetSdk()->SetDebugMode(false);
		fwscene->EnableRenderPHScene(false);
		fwscene->EnableRenderGRScene(true);
		break;
	case 'h':
		GetSdk()->SetDebugMode(true);
		fwscene->EnableRenderGRScene(false);
		fwscene->EnableRenderPHScene(true);
		break;
	case 'w':
		InitCameraView();
		break;
	case 'c': {
		calibrate();
	}
			break;
	case 'd': {
		if (displayGraphFlag) {
			displayGraphFlag = false;
			DSTR << "GRAPH DISBLED" << std::endl;
		}
		else {
			displayGraphFlag = true;
			DSTR << "GRAPH ENABLED" << std::endl;
		}
	}
	case 's': {
		this->resetObjects();
	}
			break;
	case '1': case '2': case '3': case '4': case '5': case '6':
	case '7': case '8': case '9':
		grabKey = key;

		break;

		//NUM KEYS BLOCK
	case 356: // left
	{
		pose.PosX() -= d;

	}
	break;
	case 358: // right
	{
		pose.PosX() += d;
	}
	break;
	case 357: // up
	{
		pose.PosY() += d;
	}
	break;
	case 359: // down
	{
		pose.PosY() -= d;
	}
	break;
	case ',':
		grabForce -= d;
		break;
	case '.':
		grabForce += d;
		break;
	case DVKeyCode::PAGE_UP:
		pose.PosZ() -= d;
		break;
	case DVKeyCode::PAGE_DOWN:
		pose.PosZ() += d;
		break;
	case 'f':
		bForceFeedback = !bForceFeedback;
		DSTR << "ForceFeedback: ";
		if (bForceFeedback)
		{
			DSTR << "ON\n";
		}
		else
		{
			DSTR << "OFF\n";
		}
		break;
	case 'v':
		bVibrationFeedback = !bVibrationFeedback;
		DSTR << "VibrationFeedback: ";
		if (bVibrationFeedback)
		{
			DSTR << "ON\n";
		}
		else
		{
			DSTR << "OFF\n";
		}
		break;
	case 't':
		ptimer->Stop();
		stopTimer = true;
		break;
	case 'r':
		ptimer->Start();
		break;
	case ' ':
		TimerFunc(pTimerID);
		stopTimer = true;
		break;
	case 'u':
		Vec3d g = phscene->GetGravity();
		if (g.norm() <= 0.0001)
			g = Vec3d(0.0, -9.8, 0.0);
		else
			g = Vec3d::Zero();
		phscene->SetGravity(g);
		std::cout << "g=" << g << std::endl;

	}
	grip.Step(pose, phscene->GetTimeStep());

	for (Finger& finger : grip.fingers) {
		finger.AddForce(grabForce); //
		std::cout << "grabForce:" << grabForce << std::endl;
	}
	if(!stopTimer)
		ptimer->Start();
}

//draws the force graphic on the right of the screen
void MultiFinger::displayGraph(GRRenderIf* render)
{
	Affinef view; render->GetViewMatrix(view);
	Affinef proj; render->GetProjectionMatrix(proj);
	render->SetViewMatrix(Affinef::Unit());
	render->SetProjectionMatrix(Affinef::Unit());
	render->PushLight(ld);
	render->PushModelMatrix();
	render->SetModelMatrix(Affinef::Unit());

	render->SetMaterial(Spr::GRRenderBaseIf::TMaterialSample::WHITE);
	render->EnterScreenCoordinate();
	render->SetMaterial(GRRenderIf::WHITE);
	if (count != 0) {
		static double dif = 0.1; //0.4 orignal
		for (int n = 1; n<count-1; n++) {
			render->DrawLine(Vec3d((double(n) / VIBBUF_LEN) + dif, vibBuffer[n] / 10.0, 0), 
				Vec3d((double(n+1) / VIBBUF_LEN) + dif, vibBuffer[n+1] / 10.0, 0));
		}
	}

	render->LeaveScreenCoordinate();

	render->PopModelMatrix();
	render->PopLight();
	render->SetProjectionMatrix(proj);
	render->SetViewMatrix(view);
}

void MultiFinger::Display()
{
	FWSceneIf* scene = GetWin(0)->GetScene();
	GRRenderIf* render = GetSdk()->GetRender();

	UTAutoLock LOCK(displayLock);

	render->ClearBuffer();
	render->BeginScene();
	
	if (!scene->GetGRScene() || !scene->GetGRScene()->GetCamera() || !scene->GetGRScene()->GetCamera()->GetFrame()) {
		render->SetViewMatrix(
			GetCurrentWin()->GetTrackball()->GetAffine().inv());
	}
	scene->Draw(render, true);
	render->PushLight(ld);
	render->PushModelMatrix();

	render->SetLighting(false);
	render->SetDepthTest(false);
	render->SetMaterial(Spr::GRRenderBaseIf::TMaterialSample::WHITE);
	render->EnterScreenCoordinate();
	render->SetMaterial(GRRenderIf::WHITE);
	Spr::GRFont f;
	f.height = 24;
	f.width = 10;
	render->SetFont(f);
	render->DrawFont(Vec2f(50, 50), message);
	std::string withForce = bForceFeedback ? "ON" : "OFF";
	render->DrawFont(Vec2f(1000, 100), "Force Feedback : " + withForce);
	if (practiceTrial)
		render->DrawFont(Vec2f(1000, 150), "Practice Trial");
	else
		render->DrawFont(Vec2f(1000, 150), "Trial: " + std::to_string(trialNumber) + " / " + std::to_string(20));
	render->LeaveScreenCoordinate();

	// Guide
	const float left_pos = -0.06f;
	const float right_pos = 0.04f;
	const float pinch_pos = -0.01f;
	const float lift_pos = 0.12f;
	const double target_y = target->GetFramePosition().y;
	if (target_y < 0.08 && !inContact && increaseMassState <= WAIT) {
		render->DrawFont(Vec3f(left_pos, pinch_pos, 0.0), "->");
		render->DrawFont(Vec3f(right_pos, pinch_pos, 0.0), "<- Pintch Here ");
	}
	else if(inContact && increaseMassState <= WAIT){
		render->DrawFont(Vec3f(left_pos, lift_pos, 0.0), "->");
		render->DrawFont(Vec3f(right_pos, lift_pos, 0.0), "<- Lift to Here");
	}

	render->SetLighting(true);
	render->SetDepthTest(true);
	render->PopModelMatrix();
	render->PopLight();
	render->EndScene();
	render->SwapBuffers();

	//FWApp::Display();
	
}



void MultiFinger::AtExit(){
	//this->myfile.flush();
	//this->myfile.close();  //close the DEBUG csv file at exit
}

// Initialize the position of the objects in the scene
void MultiFinger::resetObjects(){

	Quaterniond qq;
	Posed ptmp;
	target->SetVelocity(Vec3d());
	qq.FromEuler(Vec3f(Radf(0.0f), Radf(180.0f), 0.0f));
	ptmp = Posed(Vec3d(0.0f, 0.1f, 0.0f), qq);
	target->SetPose(ptmp);
	/*
	int randAngle;
	randAngle = rand() % (360 + 1);

	//left jenga
	fJenga1->SetVelocity(Vec3d());
	qq.FromEuler(Vec3f(Radf(90.0f), Radf(0.0f), 0.0f));
	ptmp = Posed(Vec3d(-0.121f, 0.021f, 0.1f), qq);
	fJenga1->SetPose(ptmp);

	//middle jenga
	//fJenga2->SetVelocity(Vec3d());
	//qq.FromEuler(Vec3f(Radf(90.0f), Radf(0.0f), 0.0f));
	//ptmp = Posed(Vec3d(-0.1f, 0.021f, 0.1f), qq);
	//fJenga2->SetPose(ptmp);

	//right jenga
	//fJenga3->SetVelocity(Vec3d());
	//qq.FromEuler(Vec3f(Radf(90.0f), Radf(0.0f), 0.0f));
	//ptmp = Posed(Vec3d(-0.080f, 0.021f, 0.1f), qq);
	//fJenga3->SetPose(ptmp);

	//phone
	fPhone->SetVelocity(Vec3d());
	qq.FromEuler(Vec3f(Radf(90.0f), Radf(0.0f), 0.0f));
	ptmp = Posed(Vec3d(0.0f, 0.045f, 0.1f), qq);
	fPhone->SetPose(ptmp);

	//hammer
	fHammer->SetVelocity(Vec3d());
	qq.FromEuler(Vec3f(Radf(0.0f), Radf(180.0f), 0.0f));
	ptmp = Posed(Vec3d(0.1f, 0.045f, 0.1f), qq);
	fHammer->SetPose(ptmp);

	//alumini cube
	fAluminio->SetVelocity(Vec3d());
	qq.FromEuler(Vec3f(Radf(0.0f), Radf(180.0f), 0.0f));
	ptmp = Posed(Vec3d(0.0f, 0.025f, -0.1f), qq);
	fAluminio->SetPose(ptmp);
	*/
}

void MultiFinger::IdleFunc() {

}

int MultiFinger::IncreaseMass(double t) {
	const double StartTime = 1.0; // [s]
	static double randomWaitTime = 0.0; // [s]
	const double Duration = 5.0; // [s]
	const double m0 = logger->condition.mass0;
	const double dmdt = logger->condition.dmdt;

	static int state = IDLE;
	double pushHeight = pushObject->GetCenterPosition().y;
	double targetHeight = target->GetCenterPosition().y + 0.12/2;
	const double offset = 0.1/ 2 + 0.01;
	const double startHeight = 0.3 + offset;
	const double v = 0.1;
	if (t < 0.0f) {
		if (state != IDLE) {
			std::cout << "Reset Increase Mass State" << std::endl;
			target->SetMass(m0);
			state = IDLE;
			randomWaitTime = 0.0;
		}
		// Move Push Object to start position
		if (pushHeight < startHeight)
			pushObject->SetVelocity(Vec3d(0.0, v, 0.0));
		else
			pushObject->SetVelocity(Vec3d(0.0, 0.0, 0.0));
	}
	else if (t < StartTime + randomWaitTime) {
		if (state == IDLE) {
			state = WAIT;
			std::cout << "[" << t << "]" << "Waiting" << StartTime << " s" << std::endl;
			randomWaitTime = 3.0 + ((double)rand() / RAND_MAX) * 3.0; // 3.0 - 6.0 [s]
			std::cout << "Random Wait Time: " << randomWaitTime << " s" << std::endl;
		}
		else if (state == WAIT && t > StartTime)
			state = RANDOM_WAIT;
		if (state == RANDOM_WAIT) {
			// Move Push Object to contact position
			if (pushHeight > targetHeight + offset)
				pushObject->SetVelocity(Vec3d(0.0, -v, 0.0));
			else if (pushHeight < targetHeight + offset)
				pushObject->SetVelocity(Vec3d(0.0, +v, 0.0));
			else {

			}
		}
	}
	else if (t < StartTime + randomWaitTime + Duration) {
		if (state != INCREASE) {
			std::cout << "[" << t << "]" << "Start Pushing Down"  <<  std::endl;
			state = INCREASE;
		}
		const double pushForce = dmdt * (t - StartTime - randomWaitTime);
		pushObject->SetVelocity(Vec3d(0.0, 0.0, 0.0));
		pushObject->SetCenterPosition(Vec3d(0.0, targetHeight + offset, 0.0));
		//target->SetMass(newMass);
		target->AddForce(Vec3d(0.0, -pushForce, 0.0));
		logger->data.mass = pushForce;
	}
	else {
		if (state == INCREASE) {
			std::cout << "End Increasing Mass" << std::endl;
			std::cout << "Final Push Force: " << dmdt * (t - StartTime - randomWaitTime) << std::endl;
			state = FINISH;
			pushObject->SetVelocity(Vec3d(0.0, 0.0, 0.0));
			pushObject->SetCenterPosition(Vec3d(0.0, targetHeight + offset, 0.0));
		}
		else if (state == FINISH) {
			pushObject->SetVelocity(Vec3d(0.0, 0.0, 0.0));
			pushObject->SetCenterPosition(Vec3d(0.0, targetHeight + offset, 0.0));
		}
	}

	return state;
}

bool MultiFinger::IsGraspForceProper(double &f) {
	// Check if the grasp force is not too large.
	const double maxForce = 6.0f; // [N]
	const double clipForce = 40.0f; // [N]
	// flexiforce value to N conversion
	const double flexiforce_to_N = grip.fingers[0].spring->GetSpring().x; //
	const double offset = 0.06/2 + 0.007/2 - 0.05; // 0.03 : target object's half dimension, 0.007 : tool's radius, 0.05 : max length
	const double force = flexiforce_to_N * (f / 6 + offset ); // 6 is spring length ratio
	if (force < maxForce) {
		//fwscene->SetSolidMaterial(GRRenderIf::TMaterialSample::WHITE, target);
		return true;
	}
	else {
		//std::cout << "Excessive Grasp Force: " << force << " N" << std::endl;
		if(force > clipForce)
			f = 6 * (clipForce / flexiforce_to_N - offset);
		//fwscene->SetSolidMaterial(GRRenderIf::TMaterialSample::RED, target);
		return false;
	}
	/* Calculate proper grasp force from mass and fricrtion parameters.
	const double minMass = logger->condition.mass0;
	const double g = 9.8;
	const double maxMass = minMass + logger->condition.dmdt * 2.0; // 1.0 s
	const double minForceRatio = minMass / maxMass;
	const double maxForceRatio = 1.0;
	const double forceRatio = (minForceRatio + maxForceRatio) * 0.5;
	double mu0 = 1.0f;
	if (logger->condition.friction_model == 0) {
		// Coulomb
		mu0 = logger->condition.coulomb.mu0;
	}
	else {
		// LuGre
		mu0 = logger->condition.lugre.A + logger->condition.lugre.B;
	}
	const double maxProperForce = forceRatio * maxMass * g / (2.0 * mu0);
	//const double minProperForce = minMass * g / (2.0 * mu0);
	return  (f < maxProperForce);
	*/
}

void MultiFinger::SetNext(bool practice) {
	increaseMassState = IDLE;
	// Close Current Log File
	logger->close();
	// Get Next Condition
	Condition c;

	static int p = 0;

	const int SEQ_NUM = sizeof(seq[0]) / sizeof(seq[0][0]);

	practiceTrial = practice;
	if (practice) {
		p = (p + 1) % CONDITION_COUNT;
		c = conditions[p];
		std::cout << "Practice Mode : Condition " << p << std::endl;
	}
	else {
		bForceFeedback = trialNumber < SEQ_NUM;
		int u = (USER_NUM + int(trialNumber / SEQ_NUM)) % 8;
		int s = trialNumber % SEQ_NUM;
		c = conditions[ seq[u][s] ];
		std::cout << "Trial " << trialNumber << ": Condition " << seq[u][s] << "ForceFeedback :" << bForceFeedback << std::endl;
		trialNumber++;
	}

	logger->condition = c;

	// Set material
	PHMaterial mat;
	mat.frictionModel = c.friction_model;
	if (mat.frictionModel >= FrictionModel::LUGRE) {
		// LuGre
		mat.bristlesSpringK = c.lugre.sigma0;
		mat.bristlesDamperD = c.lugre.sigma1;
		mat.bristlesViscosityV = c.lugre.sigma2;
		mat.timeVaryFrictionA = c.lugre.A;
		mat.timeVaryFrictionB = c.lugre.B;
		mat.timeVaryFrictionC = c.lugre.C;
		if (!practice)
			logger->open("Lugre");
		std::cout << "<<<< Lugre Condition Set >>>>" << std::endl;

	}
	else {
		// Coulomb
		mat.mu = c.coulomb.mu;
		mat.mu0 = c.coulomb.mu0;
		if (!practice)
			logger->open("Coulomb");
		std::cout << "<<<< Coulomb Condition Set >>>>" << std::endl;
	}
	target->SetMass(c.mass0);
	grip.SetMaterial(mat);

	resetObjects();

}