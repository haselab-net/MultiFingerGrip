#include "MultiFinger.h"
#include <windows.h>
#include <conio.h>
#include <Springhead.h>
#include <HumanInterface/SprHIDRUsb.h>
#include <HumanInterface/SprHIKeyMouse.h>
#include <Foundation/SprUTQPTimer.h>
#include <iomanip>
#include "Logger.hpp"


//Constructor 
MultiFinger::MultiFinger(){

	humanInterface = SPIDAR;
	fileName = "./sprfiles/scene.spr";
	pdt = 0.001f;
	posScale = 10.0;  //2.5 orignal value with 20x30 floor scene (Virgilio original) (for peta pointer)  10.0 

	//Inits the debug CSV file
	//this->myfile.open("c:\\tmp\\loco.csv");

	displayGraphFlag = false;
	logger = new Logger();
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
//This function loads the spr file and inits the scene
void MultiFinger::BuildScene(){
	
	int i = 0;
	UTRef<ImportIf> import = GetSdk()->GetFISdk()->CreateImport();	/// インポートポイントの作成
	GetSdk()->LoadScene(fileName, import);	/// ファイルのロード

	i = GetSdk()->NScene() - 1;
	phscene = GetSdk()->GetScene(i)->GetPHScene();
	phscene->SetTimeStep(pdt);

	fwscene = GetSdk()->GetScene(i);
	//fwscene->EnableRenderAxis(false, false, true);
	fwscene->EnableRenderForce(false, true);
	fwscene->EnableRenderContact(true);
	GetSdk()->SetDebugMode(true);

	grip.Build(fwscene);
	maxReach = 0.05;
	this->nsolids = phscene->NSolids();
	DSTR << "Nsolids: " << nsolids << std::endl;  //DEBUG
	PHSolidIf **solidspnt = phscene->GetSolids();
	
	PHSolidIf *floor = phscene->FindObject("soCube")->Cast();
	floor->GetShape(0)->SetStaticFriction(0.4f);

	target = phscene->FindObject("soAluminioLight")->Cast();
	SetNext();
	
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

	Vec3d pos = Vec3d(0, 0.02, 0.2);
	GetCurrentWin()->GetTrackball()->SetPosition(pos);
	Affinef af;
	af.Pos() = pos;
	Vec3d target = Vec3d(0.0, 0.05, 0);	 //focused on the tochdown zone
	GetCurrentWin()->GetTrackball()->SetTarget(target);	// カメラ初期位置の設定
}


//Calibrates the position of the grip and both pointers
void MultiFinger::calibrate() {	
	spidar->Calibration();
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

		phscene->Step();  //springhead physics step
		double a_vib = 0.0f;
		Posed pose = spidar->GetPose();
		pose.Pos() = pose.Pos()*4;
		pose.PosY() += 0.07;
		static int c = 0;
		c++;
		if (flexiforce) {
			// bad calibration! m = -1.5716   b = 2.7717  //  -2.4914    4.6105
			float volts = flexiforce->Voltage();
			double offset = 1.0; // grabForce == 0 となる位置をずらす
			grabForce = (2.0*volts) - offset;

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
		for (int i = 0; i < phscene->NContacts(); i++) {
			cp = phscene->GetContact(i);
			if (cp->GetSocketSolid() == tool || cp->GetPlugSolid() == tool){
			//if (cp->GetSocketSolid() == tool && cp->GetPlugSolid() == target ||
				//cp->GetSocketSolid() == target && cp->GetPlugSolid() == tool) {

				// Vibration
				if (logger->condition.friction_model == 0) {
					// Coulomb
					static bool prev_is_static = false;
					bool is_static = cp->IsStaticFriction();
					if (!is_static && prev_is_static) {
						// Transition from static to dynamic friction
						stickSlipTime.push_back(t);
						std::cout << "stick-slip at t=" << t << std::endl;
					}
					prev_is_static = is_static;
					const double A1 = 15.0f;
					const double A2 = 8.0f;
					const double fa1 = 15.0f;
					const double fa2 = 200.0f;
					const double decay = 200.0f;
					double forceNum = grabForce;
					if (forceNum < 0.1)
						forceNum = 0.1;
					for (float t1 : stickSlipTime) {
						if (phscene->GetCount() * pdt - t1 <= 0.2) {
							vib += A2 * sqrt(forceNum) * exp(-(t - t1) * decay)*sin(2.0f * M_PI * fa2 * (t - t1));
						}
						else {
							// Remove old stick-slip events
							stickSlipTime.erase(stickSlipTime.begin());
						}

					}
					Vec3d v, w;
					cp->GetRelativeVelocity(v, w); 
					vib += A1 * sqrt(forceNum) * v.norm() * sin(2.0f * M_PI * fa1 * t);
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
					const double fa2 = 200.0f;
					const double A1 = 1.0 / 1.5f;
					const double A2 = 5.0f;
					dT = min(-A1 * dT, 3.0f);
					double slipd = min(A2 * slip.norm(), 3.0f);
					vib = A1 * dT * sin(2.0f * M_PI * fa1 * t) + A2 * slipd * sin(2.0f * M_PI * fa2 * t);
					//std::cout << dT << "," << slipd << std::endl;
					T_p = T;
				}
				totalForce.y += vib;

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
				Vec3d cf, ct;
				cp->GetConstraintForce(cf, ct);
				logger->data.friction_force = Vec2d(cf[1], cf[2]).norm();
				logger->data.vibration_force = vib;
				logger->saveSample();

			}
		}
		if (c % 10 == 0) {
			DSTR << "grabforce: " << grabForce << ", dT" << dT << std::endl;

		}
		for (Finger& finger : grip.fingers) {

			finger.AddForce(grabForce);	//	This must be actual force sensor values. For debug purpose only first two pointers get force.
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
		double fs = 0.3, ts = 1;
		if (bForceFeedback) {
			spidar->SetForce(-fs * totalForce, -ts * totalTorque);
		}
		else {
			spidar->SetForce(Vec3d(), Vec3d());
		}

		spidar->Update(pdt);  //updates the forces displayed in SPIDAR
		//MultiFingerStep(&spidarForce);  //This function computes the lineal and rotational couplings value
		//spidar->SetForce(-spidarForce);  //This function set the force 
		
		PostRedisplay();
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
	case 27:
	case 'q':
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
	case '^':
		SetNext();
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
	GRRenderIf* render = GetSdk()->GetRender();

	if (displayGraphFlag) {
		displayGraph(render);
	}

	FWApp::Display();
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
	ptmp = Posed(Vec3d(0.0f, 0.035f, 0.0f), qq);
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

void MultiFinger::SetNext() {
	// Close Current Log File
	logger->close();
	// Get Next Condition
	Condition c;
	if (rand() % 2) {
		c = con_lugre;
	}
	else {
		c = con_coulomb;
	}

	logger->condition = c;

	// Set material
	PHMaterial mat;
	mat.frictionModel = c.friction_model;
	if (mat.frictionModel == 1) {
		// LuGre
		mat.bristlesSpringK = c.lugre.sigma0;
		mat.bristlesDamperD = c.lugre.sigma1;
		mat.bristlesViscosityV = c.lugre.sigma2;
		mat.timeVaryFrictionA = c.lugre.A;
		mat.timeVaryFrictionB = c.lugre.B;
		mat.timeVaryFrictionC = c.lugre.C;
		logger->open("Lugre");
		std::cout << "<<<< Lugre Condition Set >>>>" << std::endl;

	}
	else {
		// Coulomb
		mat.mu = c.coulomb.mu;
		mat.mu0 = c.coulomb.mu0;
		logger->open("Coulomb");
		std::cout << "<<<< Coulomb Condition Set >>>>" << std::endl;
	}

	grip.SetMaterial(mat);

	resetObjects();

}