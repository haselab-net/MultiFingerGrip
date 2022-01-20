#include "MultiFinger.h"
#include <windows.h>
#include <conio.h>
#include <Springhead.h>
#include <HumanInterface/SprHIDRUsb.h>
#include <HumanInterface/SprHIKeyMouse.h>
#include <Foundation/SprUTQPTimer.h>
#include <iomanip>



//Constructor 
MultiFinger::MultiFinger(){

	humanInterface = SPIDAR;
	fileName = "./sprfiles/scene.spr";
	pdt = 0.001f;
	posScale = 10.0;  //2.5 orignal value with 20x30 floor scene (Virgilio original) (for peta pointer)  10.0 

	//Inits the debug CSV file
	//this->myfile.open("c:\\tmp\\loco.csv");

	displayGraphFlag = true;
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
	//fwscene->EnableRenderAxis();

	grip.Build(fwscene);


	GetSdk()->GetScene(i)->EnableRenderHaptic();
	hapscene = phscene->GetHapticEngine();
	//hapscene->EnableHapticEngine(true);

	PHHapticEngineIf* he = phscene->GetHapticEngine();	// 力覚エンジンをとってくる
	PHHapticEngineDesc hd;
	//he->EnableHapticEngine(true);						// 力覚エンジンの有効化
	//he->SetHapticEngineMode(PHHapticEngineDesc::SINGLE_THREAD);
	maxReach = 0.05;
	this->nsolids = phscene->NSolids();
	DSTR << "Nsolids: " << nsolids << std::endl;  //DEBUG
	PHSolidIf **solidspnt = phscene->GetSolids();
	
	//set the objects that are going to be the Tools (BINOD)
	fTool0 = phscene->FindObject("soTool0")->Cast();
	fTool1 = phscene->FindObject("soTool1")->Cast();
	fTool2 = phscene->FindObject("soTool2")->Cast();

	defaultCenterPose.Pos() = Vec3d(0.0, 0.15, 0.0);

	//defaultPose1.Pos() = Vec3d(maxReach, 0.15, 0.0);
	//fTool0->SetMass(0.010);
	//fTool0->SetInertia(fTool0->GetShape(0)->CalcMomentOfInertia() * (float)(fTool0->GetMass() / fTool0->GetShape(0)->CalcVolume()));	// 慣性テンソルの設定

	//defaultPose2.Pos() = Vec3d(maxReach, -0.15, 0.0);
	//fTool1->SetMass(0.010);  //0.0005 original value
	//fTool1->SetInertia(fTool1->GetShape(0)->CalcMomentOfInertia() * (float)(fTool1->GetMass() / fTool1->GetShape(0)->CalcVolume()));	// 慣性テンソルの設定
	
	//Default pose for pointer 3
	//defaultPose3.Pos() = Vec3d(-maxReach, -0.15, 0.0);
	//fTool2->SetMass(0.010);  //0.0005 original value
	//fTool2->SetInertia(fTool2->GetShape(0)->CalcMomentOfInertia() * (float)(fTool2->GetMass() / fTool2->GetShape(0)->CalcVolume()));	// 慣性テンソルの設定
	
	//fTool0->SetGravity(false);
	//fTool1->SetGravity(false);
	//fTool2->SetGravity(false);
	
	//Used to locate the object in the right orientation, from Euler angles
	//Quaterniond qq;
	//qq.FromEuler(Vec3f(Radf(0.0f) , 0.0f, Radf(90.0f)));
	//Posed pp = Posed(Vec3d(0,0,0), qq);
	//DSTR << pp.OriW() << "," << pp.OriX() << "," << pp.OriY() << "," << pp.OriZ() << std::endl;

	PHSolidIf *floor = phscene->FindObject("soCube")->Cast();
	//fTool0->GetShape(0)->SetStaticFriction(1000.0f);
	//fTool1->GetShape(0)->SetStaticFriction(1000.0f);
	//fTool2->GetShape(0)->SetStaticFriction(1000.0f);
	//fTool0->GetShape(0)->SetDynamicFriction(1000.0f);
	//fTool1->GetShape(0)->SetDynamicFriction(1000.0f);
	//fTool2->GetShape(0)->SetDynamicFriction(1000.0f);
	
	floor->GetShape(0)->SetStaticFriction(0.4f);
	
	
	//define jenga object properties
	fJenga1 = phscene->FindObject("soJenga1")->Cast();
	fJenga1->GetShape(0)->SetDensity(357.142f);  //non specific value try and error
	fJenga1->CompInertia();
	fJenga1->GetShape(0)->SetStaticFriction(0.7f);
	fJenga1->GetShape(0)->SetDynamicFriction(0.7f);
	//DSTR << "jenga mass: " << fJenga1->GetMass() << std::endl;  //debug
	//DSTR << "jenga volume: " << fJenga1->GetShape(0)->CalcVolume() << std::endl;  //debug

	//fJenga2 = phscene->FindObject("soJenga2")->Cast();
	//fJenga2->GetShape(0)->SetDensity(357.142);  //non specific value try and error
	//fJenga2->CompInertia();
	//fJenga2->GetShape(0)->SetStaticFriction(0.7);
	//fJenga2->GetShape(0)->SetDynamicFriction(0.7);

	//fJenga3 = phscene->FindObject("soJenga3")->Cast();
	//fJenga3->GetShape(0)->SetDensity(357.142); //non specific value try and error
	//fJenga3->CompInertia();
	//fJenga3->GetShape(0)->SetStaticFriction(0.7);
	//fJenga3->GetShape(0)->SetDynamicFriction(0.7);

	//defining the cellphone density and mass
	fPhone = phscene->FindObject("soPhone")->Cast();
	fPhone->GetShape(0)->SetDensity(677);  // non specific value try and error
	fPhone->CompInertia();
	fPhone->GetShape(0)->SetStaticFriction(0.7f);
	fPhone->GetShape(0)->SetDynamicFriction(0.7f);
	DSTR << "phone mass: " << fPhone->GetMass() << std::endl;  //debug
	DSTR << "phone volume" << fPhone->GetShape(0)->CalcVolume() << std::endl;  //debug
	
	//defining the hammer density and mass
	fHammer = phscene->FindObject("soHammer")->Cast();
	fHammer->GetShape(0)->SetDensity(1500); //non specific value try and error  1012.25
	fHammer->GetShape(1)->SetDensity(244.90f);  //non specific value try and error
	fHammer->GetShape(0)->SetStaticFriction(0.70f);
	fHammer->GetShape(0)->SetDynamicFriction(0.7f);
	fHammer->GetShape(1)->SetStaticFriction(0.7f);
	fHammer->GetShape(1)->SetDynamicFriction(0.7f);
	fHammer->CompInertia();
	//DSTR << "hammer mass: " << fHammer->GetMass() << std::endl;  //debug
	//DSTR << "head volume: " << fHammer->GetShape(0)->CalcVolume() << std::endl;  //debug
	//DSTR << "stick volume: " << fHammer->GetShape(1)->CalcVolume() << std::endl; //debug 

	//aluminium block  mass and density  64cm^3 * iron density (7.87gr) = 200
	fAluminio = phscene->FindObject("soAluminio")->Cast();
	fAluminio->GetShape(0)->SetDensity(1355);  //non specific value try and error
	fAluminio->CompInertia();
	fAluminio->GetShape(0)->SetStaticFriction(0.7f);
	fAluminio->GetShape(0)->SetDynamicFriction(0.7f);
	DSTR << "aluminio vol: " << fAluminio->GetShape(0)->CalcVolume() << std::endl;  //debug
	DSTR << "aluminio mass: " << fAluminio->GetMass() << std::endl;   //debug

}

//Inits SPIDAR and calibrates the pointer position
void MultiFinger::InitHapticInterface(){
	HISdkIf* hiSdk = GetSdk()->GetHISdk();

	bool bFoundCy = false;
	if (humanInterface == SPIDAR){
		// x64
		DRCyUsb20Sh4Desc cyDesc;
		for (int i = 0; i<10; ++i){
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
	else if (humanInterface == XBOX){
		spidar = hiSdk->CreateHumanInterface(HIXbox360ControllerIf::GetIfInfoStatic())->Cast();
	}
	else if (humanInterface == FALCON){
		spidar = hiSdk->CreateHumanInterface(HINovintFalconIf::GetIfInfoStatic())->Cast();
		spidar->Init(NULL);
	}
	/*//The port is 3 because the sensor is connected to the port 3 on the Spidar's AD Converter
	if (bFoundCy) {
		flexiforce = hiSdk->RentVirtualDevice(DVAdIf::GetIfInfoStatic(), "", 3)->Cast();
	}
	else {
		flexiforce = hiSdk->RentVirtualDevice(DVAdIf::GetIfInfoStatic(), "", 2)->Cast();
	}
	if (flexiforce && 1700 < flexiforce->Digit() && flexiforce->Digit() < 1900) {
		flexiforce = NULL;
	}*/
}


void MultiFinger::InitCameraView(){

	Vec3d pos = Vec3d(0, 0.09, 0.3);
	GetCurrentWin()->GetTrackball()->SetPosition(pos);
	Affinef af;
	af.Pos() = pos;
	Vec3d target = Vec3d(0.0, 0.05, 0);	 //focused on the tochdown zone
	GetCurrentWin()->GetTrackball()->SetTarget(target);	// カメラ初期位置の設定
}


//Calibrates the position of the grip and both pointers
void MultiFinger::calibrate() {	
	/*
	fTool0->SetFramePosition(defaultPose1.Pos());
	fTool0->SetOrientation(spidar->GetOrientation());
	fTool0->SetVelocity(Vec3d(0.0, 0.0, 0.0));
	fTool0->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));

	fTool1->SetFramePosition(defaultPose2.Pos());
	fTool1->SetOrientation(spidar->GetOrientation());
	fTool1->SetVelocity(Vec3d(0.0, 0.0, 0.0));
	fTool1->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));

	fTool2->SetFramePosition(defaultPose3.Pos());
	fTool2->SetOrientation(spidar->GetOrientation());
	fTool2->SetVelocity(Vec3d(0.0, 0.0, 0.0));
	fTool2->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
	*/
	// Check this one (if only one or all pointers)
	//fTool1->SetDynamical(true);
	//fTool1->SetDynamical(true);
	//fTool2->SetDynamical(true);
	spidar->Calibration();
}

//This multimedia thread handles the haptic (6DOF virtual coupling pointers) and physics simulation (Springhead)
// This is for multifinger grasping (BINOD).
/*
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
			std::cout << cps << std::endl;
			lastCounted = now;
			cycle = 0;
			int a = 0;
			}
			cycle++;
		}
		UTAutoLock LOCK(displayLock);

		phscene->Step();  //springhead physics step
		
		Posed pose = spidar->GetPose();
		pose.Pos() = pose.Pos()*4;
		pose.PosY() += 0.02;
		grip.Step(pose, phscene->GetTimeStep());	//	this will be actual code.
		
				
		//force reading from each sensor (Binod)
		Vec3d vecgrabForce0;
		Vec3d vecgrabForce1;
		
		if (uartMotorDriver->NForce() > 0) {
			for (size_t i = 0; i < uartMotorDriver->NForce(); ++i) {
				//DSTR << uartMotorDriver->ReadForce((int)i) << ",";
				//DSTR << std::endl;

				//if (uartMotorDriver->ReadForce((int)i) <= 2) {

					//grabforce1 = (double)(uartMotorDriver->ReadForce(1)/800 +0.2);
				grabForce0 = (double)(uartMotorDriver->ReadForce(1) *0.0994 - 11.791);
				grabForce1 = (double)(uartMotorDriver->ReadForce(3) *0.0994 - 10.723);
				
				//grabForce = (double)(( grabForce0 + grabForce1)/2);
				vecgrabForce0 = Vec3d(-grabForce0, 0.0, 0.0);
				vecgrabForce1 = Vec3d(0.0, -grabForce1, 0.0);
				grabForce = ((grabForce0 + grabForce1)/2);

				//grabForce = (double)(uartMotorDriver->ReadForce((int)i) * -0.009914 + 2.1105);
				DSTR << "Force: " << grabForce << "," << "Force1: " << grabForce0 << "," << "Force2: " << grabForce1 << ";";;
				DSTR << std::endl;
								
				}
				
			}
		
		
		Vec3d totalForce, totalTorque;
		for (Finger& finger : grip.fingers) {
			if(finger.GetIndex() < 0) finger.AddForce(-grabForce);	//	This must be actual force sensor values. For debug purpose only first two pointers get force.
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
		spidar->SetForce(-fs*totalForce, -ts*totalTorque);
		//DSTR << "Total = f:" << totalForce << " t:" << totalTorque << std::endl;

		
		//adding force feedback onto the SPIDAR finger (Binod)
		Vec3d grabforce0;
		Vec3d grabforce1;
		Vec3d grabforce2;
				
		if (grabForce0 && grabForce1 <= 5) {   //nearly 3 is the original values
				
		//used to separate the pointers, once they had been in contact
				grabforce0 = vecgrabForce0 * 0.1;
				grabforce1 = vecgrabForce0 * 0.1;
				grabforce2 = ((grabforce0 + grabforce1 + grabforce2) * 0.1);

			}
			else {
				//If the force is bellow the threshold value, then apply the force into the haptic pointers
				grabforce0 = vecgrabForce0;
				grabforce1 = vecgrabForce1;
				grabforce2 = (grabforce0 + grabforce1 + totalForce);
			}

			//update the haptic pointers
			fTool0->AddForce(grabforce0);
			fTool1->AddForce(grabforce1);
			fTool2->AddForce(grabforce2);

		//	Following two lines fixs the tool[0] to see coupling force for debugging.
		//	grip.fingers[0].tool->SetFramePosition(Vec3d(0.05, 0.2, 0));
		//	grip.fingers[0].tool->SetVelocity(Vec3d(0, 0, 0));

		//	show current and target position of slider joint 0.
		//for (int i = 0; i<grip.fingers.size(); ++i) {
			//DSTR << "Slider" << i << " pos:" << grip.fingers[i].slider->GetPosition()
				//<< " target:" << grip.fingers[i].slider->GetTargetPosition()
				//<< " length:" << grip.fingers[i].length << std::endl;
		//}
		
		//for (Finger& finger : grip.fingers) {
			//Vec3d f, t;
			//finger.slider->GetConstraintForce(f, t);
			//DSTR << "Slider" << finger.GetIndex() << " vel:" << finger.tool->GetVelocity() << finger.tool->GetAngularVelocity() << std::endl;
			//DSTR << " pos:" << finger.tool->GetCenterPosition() << finger.tool->GetOrientation().RotationHalf() << " f:" << f << " t:" << t << std::endl;
		//}
		
		
		spidar->Update(pdt);  //updates the forces displayed in SPIDAR
						
		PostRedisplay();
	}
	else {
		return;
	}
}
*/
///*
// This is for twofinger grasping with single force sensor (BINOD).
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
			std::cout << cps << std::endl;
			lastCounted = now;
			cycle = 0;
			int a = 0;
			}
			cycle++;
		}
		UTAutoLock LOCK(displayLock);

		phscene->Step();  //springhead physics step

		Posed pose = spidar->GetPose();
		pose.Pos() = pose.Pos()*4;
		pose.PosY() += 0.02;

		Vec3d totalForce, totalTorque;
		for (Finger& finger : grip.fingers) {
			Vec6d couplingForce = finger.spring->GetMotorForce();
			//DSTR << "c" << finger.GetIndex() << " f=" << couplingForce << std::endl;
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
		spidar->SetForce(-fs*totalForce, -ts*totalTorque);
		//DSTR << "Total = f:" << totalForce << " t:" << totalTorque << std::endl;

		//	compute forces from user's finger to grip ---------------------
		//	force reading from each sensor (Binod)
		Vec3d vecgrabForce0; // converting scalar quantity into vector

		int forceChannels[] = { 1, 3 };
		#define NFORCES		(sizeof(forceChannels) / sizeof(forceChannels[0]) + 1)
		double forces[NFORCES];
		Vec3d vectorForces[NFORCES];
		for (size_t i = 0; i < NFORCES-1; ++i) {
			int ch = forceChannels[i];
			if (ch < uartMotorDriver->NForce()) {
				forces[i] = (double)(uartMotorDriver->ReadForce(ch) * 0.0994 - 12.9135);
				vectorForces[i] = forces[i] * (grip.gripDevice->GetOrientation() * grip.fingers[i].direction);
			}
		}	
		vectorForces[NFORCES - 1] = totalForce;
		for (size_t i = 0; i < NFORCES - 1; ++i) {
			vectorForces[NFORCES - 1] -= vectorForces[i];
		}
		forces[NFORCES - 1] = vectorForces[NFORCES - 1].norm();
		for (size_t i = 0; i < grip.fingers.size(); ++i) {
			grip.fingers[i].AddForce(forces[i] - 2.0);	//	This must be actual force sensor values. For debug purpose only first two pointers get force.
		}			
		grip.Step(pose, phscene->GetTimeStep());

		spidar->Update(pdt);  //updates the forces displayed in SPIDAR

		PostRedisplay();
		}
		else {
		return;
	}
}
//*/


//catches keyboard events
void MultiFinger::Keyboard(int key, int x, int y){
	if (ptimer){
		while (!ptimer->Stop());
	}
	int spKey = key - 0x100;
	switch (key){
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
		case 'd': {
			if (displayGraphFlag) {
				displayGraphFlag = false;
				DSTR << "GRAPH DISBLED" << std::endl;
			}else {
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

		}
			break;
		case 358: // right
		{
			

		}
			break;
		case 357: // up
		{
			
		}
			break;
		case 359: // down
		{
			
		}
			break;
	}
	
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

//Used to graphically debug the program
void MultiFinger::Display() 
{
	GRRenderIf* render = GetSdk()->GetRender();

	if (displayGraphFlag){
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

	int randAngle;
	randAngle = rand() % (360 + 1);

	Quaterniond qq;
	Posed ptmp;

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
}
