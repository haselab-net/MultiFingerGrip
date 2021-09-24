#include "TwoFinger.h"
#include <windows.h>

//Constructor 
TwoFinger::TwoFinger(){

	humanInterface = SPIDAR;
	fileName = "./sprfiles/scene.spr";
	pdt = 0.001f;
	srand((unsigned)time(NULL));
	posScale = 10.0;  //2.5 orignal value with 20x30 floor scene (Virgilio original) (for peta pointer)  10.0 

	//Inits the debug CSV file
	//this->myfile.open("c:\\tmp\\loco.csv");

	displayGraphFlag = true;
}

//main function of the class
void TwoFinger::Init(int argc, char* argv[]){
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

	//petaGrab = new GrabPointer(phscene, fPointer1);
	count = 0;
	delay = 0;

}
//This function loads the spr file and inits the scene
void TwoFinger::BuildScene(){
	
	int i = 0;
	UTRef<ImportIf> import = GetSdk()->GetFISdk()->CreateImport();	/// インポートポイントの作成
	GetSdk()->LoadScene(fileName, import);	/// ファイルのロード

	i = GetSdk()->NScene() - 1;
	phscene = GetSdk()->GetScene(i)->GetPHScene();
	phscene->SetTimeStep(pdt);

	fwscene = GetSdk()->GetScene(i);

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

	//set the objects that are going to be the pointers
	fPointer1 = phscene->FindObject("soPointer1")->Cast();
	fPointer2 = phscene->FindObject("soPointer2")->Cast();
	
	defaultCenterPose.Pos() = Vec3d(0.0, 0.15, 0.0);
	defaultPose1.Pos() = Vec3d(maxReach, 0.15, 0.0);
	fPointer1->SetMass(0.010);
	fPointer1->SetInertia(fPointer1->GetShape(0)->CalcMomentOfInertia() * (float)(fPointer1->GetMass() / fPointer1->GetShape(0)->CalcVolume()));	// 慣性テンソルの設定

	defaultPose2.Pos() = Vec3d(-maxReach, 0.15, 0.0);
	fPointer2->SetMass(0.010);  //0.0005 original value
	fPointer2->SetInertia(fPointer2->GetShape(0)->CalcMomentOfInertia() * (float)(fPointer2->GetMass() / fPointer2->GetShape(0)->CalcVolume()));	// 慣性テンソルの設定
	fPointer1->SetGravity(false);
	fPointer2->SetGravity(false);

	//Used to locate the object in the right orientation, from Euler angles
	//Quaterniond qq;
	//qq.FromEuler(Vec3f(Radf(0.0f) , 0.0f, Radf(90.0f)));
	//Posed pp = Posed(Vec3d(0,0,0), qq);
	//DSTR << pp.OriW() << "," << pp.OriX() << "," << pp.OriY() << "," << pp.OriZ() << std::endl;

	PHSolidIf *floor = phscene->FindObject("soCube")->Cast();
	fPointer1->GetShape(0)->SetStaticFriction(1000.0f);
	fPointer2->GetShape(0)->SetStaticFriction(1000.0f);
	fPointer1->GetShape(0)->SetDynamicFriction(1000.0f);
	fPointer2->GetShape(0)->SetDynamicFriction(1000.0f);
	floor->GetShape(0)->SetStaticFriction(0.4f);
	floor->GetShape(0)->SetDynamicFriction(0.4f);
	
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
void TwoFinger::InitHapticInterface(){
	HISdkIf* hiSdk = GetSdk()->GetHISdk();

	bool bFoundCy = false;
	if (humanInterface == SPIDAR){
		//// x86
		//DRUsb20SimpleDesc usbSimpleDesc;
		//hiSdk->AddRealDevice(DRUsb20SimpleIf::GetIfInfoStatic(), &usbSimpleDesc);
		//DRUsb20Sh4Desc usb20Sh4Desc;
		//for(int i=0; i< 10; ++i){
		//	usb20Sh4Desc.channel = i;
		//	hiSdk->AddRealDevice(DRUsb20Sh4If::GetIfInfoStatic(), &usb20Sh4Desc);
		//}
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
		hiSdk->AddRealDevice(DRUARTMotorDriverIf::GetIfInfoStatic(), &uartDesc);
		hiSdk->AddRealDevice(DRKeyMouseWin32If::GetIfInfoStatic());
		hiSdk->Print(DSTR);
		hiSdk->Print(std::cout);

		spg = hiSdk->CreateHumanInterface(HISpidarGIf::GetIfInfoStatic())->Cast();
		if (bFoundCy) {
			spg->Init(&HISpidarGDesc("SpidarG6X3R")); //Original SPIDARG6
			std::cout << "Init SpidarG6X3R" << std::endl;
		}
		else {
			spg->Init(&HISpidarGDesc("SpidarG6X4R"));	//	low price X SPIDAR
			std::cout << "Init SpidarG6X4R" << std::endl;
			DSTR << "Init SpidarG6X4R" << std::endl;
		}
		spg->Calibration();
		spidar = spg->Cast();
	}
	else if (humanInterface == XBOX){
		spg = hiSdk->CreateHumanInterface(HIXbox360ControllerIf::GetIfInfoStatic())->Cast();
	}
	else if (humanInterface == FALCON){
		spg = hiSdk->CreateHumanInterface(HINovintFalconIf::GetIfInfoStatic())->Cast();
		spg->Init(NULL);
	}
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
}

void TwoFinger::InitCameraView(){

	Vec3d pos = Vec3d(0, -0.4, -0.5);
	GetCurrentWin()->GetTrackball()->SetPosition(pos);
	Affinef af;
	af.Pos() = pos;
	af.LookAt(Vec3d(0, 0, 0), Vec3d(0, 1, 0));
	Quaterniond ori;
	ori.FromMatrix(af.Rot());
	GetCurrentWin()->GetTrackball()->SetOrientation(ori);

	Vec3d target = Vec3d(0.0, 0.05, -0.1);	 //focused on the tochdown zone
	GetCurrentWin()->GetTrackball()->SetTarget(target);	// カメラ初期位置の設定
}

void TwoFinger::TwoFingerStep(Vec3f *spidarForce, Vec3f *spidarTorque)
{
	double linSpring = (_stricmp(spidar->GetSpidarType(), "SpidarG6X4R") == 0 || _stricmp(spidar->GetSpidarType(), "SpidarG6X4L") == 0)
		? 50: 100;//1500;
	double linDamper = 3;//5;     //3 my value
	double rotSpring = 0.01;//10;      //torsional spring   original value 0.001
	double rotDamper = 0.1*0.001; // 0.1*rotSpring;   //torsional damper  orignal values 0.1*rotSpring

	Vec3d pos1 = fPointer1->GetPose().Pos();
	Vec3d pos2 = fPointer2->GetPose().Pos();
	Vec3d vel1 = fPointer1->GetVelocity();
	Vec3d vel2 = fPointer2->GetVelocity();

	Vec3d centerPos = Vec3d((pos1.X() + pos2.X()) / 2, (pos1.Y() + pos2.Y()) / 2, (pos1.Z() + pos2.Z()) / 2);  //middle point between the solids
	Vec3d radius1 = pos1 - centerPos;
	Vec3d radius2 = pos2 - centerPos;  //distance from the middle point to the pointer
	Quaterniond spidarRot = spidar->GetOrientation();
	Vec3d unitVectorX = (spidarRot * Vec3d(1, 0, 0)).unit();  //gets a X axis component of the spidar orientation
	Posed localSpidarPose1 = Posed(1, 0, 0, 0, radius1 * unitVectorX, 0, 0);   //gets the local orientation on X axis and match it with spidar orientation
	Posed localSpidarPose2 = Posed(1, 0, 0, 0, radius2 * unitVectorX, 0, 0);
	Posed spidarPose = spidar->GetPose();
	spidarPose.Pos() *= posScale;   //scales the spidar device position
	Posed spidarPose1 = spidarPose * localSpidarPose1;  //locate pointers accordingly to spidar position, on the local orientation axis
	Posed spidarPose2 = spidarPose * localSpidarPose2;
	Vec3d spidarRadius1 = spidarPose1.Pos() - spidarPose.Pos();   //global distance between the pointers and the spidarposition (after scale)
	Vec3d spidarRadius2 = spidarPose2.Pos() - spidarPose.Pos();

	Vec3d spidarVel = spidar->GetVelocity();
	Vec3d spidarAngVel = spidar->GetAngularVelocity();

	//gets the pointer velocity
	Vec3d spidarVel1 = spidarVel + (spidarAngVel % spidarRadius1); // spidar angular vel. cross product to pointer distance to the center, plus pointer velocity
	Vec3d spidarVel2 = spidarVel + (spidarAngVel % spidarRadius2); // (Cross Multiplication (TO GET NORMAL VECTOR)) 

	Quaterniond rotationDelta1 = fPointer1->GetOrientation() * spidarPose1.Ori().Inv(); //Mix orientation of the spidar pointer and the solids
	Quaterniond rotationDelta2 = fPointer2->GetOrientation() * spidarPose2.Ori().Inv();
	Vec3d rotAngle1 = rotationDelta1.Rotation();  //gets the rotation vector
	Vec3d rotAngle2 = rotationDelta2.Rotation();

	//Gets the lineal coupling force 
	Vec3d couplingForce1 = (-linSpring * ((pos1 - defaultCenterPose.Pos()) - spidarPose1.Pos()) - linDamper * (vel1 - spidarVel1));
	Vec3d couplingForce2 = (-linSpring * ((pos2 - defaultCenterPose.Pos()) - spidarPose2.Pos()) - linDamper * (vel2 - spidarVel2));

	//the rotational coupling torque 
	Vec3d couplingTorque1 = -rotSpring * rotAngle1 - rotDamper * (fPointer1->GetAngularVelocity() - spidarAngVel);
	Vec3d couplingTorque2 = -rotSpring * rotAngle2 - rotDamper * (fPointer2->GetAngularVelocity() - spidarAngVel);

	Vec3f spidarTorque1 = couplingTorque1 + couplingForce1 % spidarRadius1;
	Vec3f spidarTorque2 = couplingTorque2 + couplingForce2 % spidarRadius2;

	fPointer1->AddForce(couplingForce1);
	fPointer2->AddForce(couplingForce2);
	fPointer1->AddTorque(couplingTorque1);  
	fPointer2->AddTorque(couplingTorque2);  

	//This if is always true
	static int c = 0;
	c++;
	if (flexiforce) {
		// bad calibration! m = -1.5716   b = 2.7717  //  -2.4914    4.6105
		float volts = flexiforce->Voltage();
		if (c % 100 == 0) {
			DSTR << "force: " << volts << std::endl;
		}
		grabForce = (volts*-2.4914 + 4.4105);
	}
	else {
		grabForce = (grabKey - '1') * 0.2;
	}

	Vec3d grabforce1;
	Vec3d grabforce2;

	double distance = (pos1 - pos2).norm();
	//grabForce = maxReach * 2.5;
	if (grabForce <= 0.2) {   //0.03  original Virgilio value
		if (distance < maxReach * 2) {	//	
		//used to separate the pointers, once they had been in contact
			grabforce1 = radius1.unit() * 0.1;
			grabforce2 = radius2.unit() * 0.1;
		}
	}
	else {
	//If the force is bellow the threshold value, then apply the force into the haptic pointers
		grabforce1 = -radius1.unit() * grabForce;
		grabforce2 = -radius2.unit() * grabForce;
	}

	//update the haptic pointers
	fPointer1->AddForce(grabforce1);  
	fPointer2->AddForce(grabforce2);

	//buffer used to draw the force sensor force into de screen
	if (delay == 50) {
		if (count < VIBBUF_LEN) {
			vibBuffer[count] = grabForce;//振動の変位(表示用)
			count++;
		}
		else {
			vibBuffer[count] = grabForce;//振動の変位(表示用)
			count = 0;
		}
		delay = 0;
	}
	else  { delay++; }


	//This block displays the calculated force and torque in SPIDAR
	//The ifs avoid displaying to much force on the haptic grip
	float maxForce = 4.0f;
	float maxTorque = 4.0f;
	*spidarForce = couplingForce1 + couplingForce2;
	*spidarTorque = spidarTorque1 + spidarTorque2;
	if (spidarForce->X() < 0)
		spidarForce->X() = max(spidarForce->X(), -maxForce);
	else
		spidarForce->X() = min(spidarForce->X(), maxForce);

	if (spidarForce->Y() < 0)
		spidarForce->Y() = max(spidarForce->Y(), -maxForce);
	else
		spidarForce->Y() = min(spidarForce->Y(), maxForce);

	if (spidarForce->Z() < 0)
		spidarForce->Z() = max(spidarForce->Z(), -maxForce);
	else
		spidarForce->Z() = min(spidarForce->Z(), maxForce);

	if (spidarTorque->X() < 0)
		spidarTorque->X() = max(spidarTorque->X(), -maxTorque);
	else
		spidarTorque->X() = min(spidarTorque->X(), maxTorque);

	if (spidarTorque->Y() < 0)
		spidarTorque->Y() = max(spidarTorque->Y(), -maxTorque);
	else
		spidarTorque->Y() = min(spidarTorque->Y(), maxTorque);

	if (spidarTorque->Z() < 0)
		spidarTorque->Z() = max(spidarTorque->Z(), -maxTorque);
	else
		spidarTorque->Z() = min(spidarTorque->Z(), maxTorque);
	
}

//Calibrates the position of the grip and both pointers
void TwoFinger::calibrate() {
	
	fPointer1->SetFramePosition(defaultPose1.Pos());
	fPointer1->SetOrientation(spidar->GetOrientation());
	fPointer1->SetVelocity(Vec3d(0.0, 0.0, 0.0));
	fPointer1->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));

	fPointer2->SetFramePosition(defaultPose2.Pos());
	fPointer2->SetOrientation(spidar->GetOrientation());
	fPointer2->SetVelocity(Vec3d(0.0, 0.0, 0.0));
	fPointer2->SetAngularVelocity(Vec3d(0.0, 0.0, 0.0));
	fPointer2->SetDynamical(true);

	spg->Calibration();
}

//This multimedia thread handles the haptic (6DOF virtual coupling pointers) and physics simulation (Springhead)
void TwoFinger::TimerFunc(int id){

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

		Vec3f spidarForce;
		Vec3f spidarTorque;

		phscene->Step();  //springhead physics step
		spidar->Update(pdt);  //updates the forces displayed in SPIDAR

		TwoFingerStep(&spidarForce, &spidarTorque);  //This function computes the lineal and rotational couplings value
		spidar->SetForce(-spidarForce, -spidarTorque);  //This function set the force 

		PostRedisplay();
	}
	else {
		return;
	}
}

//catches keyboard events
void TwoFinger::Keyboard(int key, int x, int y){
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
		case 'x': {
			Posed p1 = fPointer1->GetPose();
			Posed p2 = fPointer2->GetPose();
			p1.PosX() = p1.PosX() + 0.1;
			p2.PosX() = p2.PosX() - 0.1;
			fPointer1->SetPose(p1);
			fPointer2->SetPose(p2);
		}
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
void TwoFinger::displayGraph(GRRenderIf* render)
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
void TwoFinger::Display() 
{
	GRRenderIf* render = GetSdk()->GetRender();

	if (displayGraphFlag){
		displayGraph(render);
	}

	FWApp::Display();
}


void TwoFinger::AtExit(){
	//this->myfile.flush();
	//this->myfile.close();  //close the DEBUG csv file at exit
}

// Initialize the position of the objects in the scene
void TwoFinger::resetObjects(){

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
