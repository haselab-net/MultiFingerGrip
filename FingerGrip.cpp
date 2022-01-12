#include "FingerGrip.h"

void Finger::AddForce(double f) {
	force += f;
}
void Finger::Step(Posed gripPose, double dt) {
	length += force * dt / mass;
	force = 0;
	LimitLength();
	Posed pose = gripPose * Posed::Trn(position + length*direction);
	device->SetPose(pose);
}


FingerGrip::FingerGrip() {
	fingers.resize(3);
	for (int i = 0; i < fingers.size(); ++i) {
		Finger& finger = fingers[i];
		finger.index = i;
		finger.direction = Quaterniond::Rot(Rad(180), 'z') * Quaterniond::Rot(Rad(120 * i), 'z') * Vec3d(1, 0, 0);
		finger.position = Quaterniond::Rot(Rad(120 * i), 'z') * Vec3d(finger.maxLength, 0, 0);
	}
}
void FingerGrip::Build(FWSceneIf* fwScene) {
	soGrip = fwScene->GetPHScene()->CreateSolid();
	soGrip->SetGravity(false);
	CDSphereDesc sd;
	sd.radius = 0.01;
	CDShapeIf* shape = fwScene->GetSdk()->GetPHSdk()->CreateShape(sd);
	soGrip->AddShape(shape);
	//	build fingers
	for (Finger& finger : fingers) {
		finger.Build(fwScene);
	}

	Step(Posed::Trn(0, 0.2, 0), 0.01);	//	set device position

	//	set collision mode and tool pose
	fwScene->GetPHScene()->SetContactMode(soGrip, PHSceneDesc::MODE_NONE);
	for (Finger& finger : fingers) {
		fwScene->GetPHScene()->SetContactMode(finger.device, PHSceneDesc::MODE_NONE);
		finger.tool->SetPose(finger.device->GetPose());
	}
}


void FingerGrip::Step(Posed p, double dt) {
	pose = p;
	soGrip->SetPose(pose);
	for (Finger& finger: fingers) {
		finger.Step(p, dt);
	}
}


void Finger::Build(FWSceneIf* fwScene) {
	PHSceneIf* phScene = fwScene->GetPHScene();
	ostringstream toolName;
	toolName << "soTool" << index;
	tool = phScene->FindObject(toolName.str().c_str())->Cast();
	if (!tool) {
		tool = fwScene->GetPHScene()->CreateSolid();
	}
	tool->SetGravity(false);

	ostringstream deviceName;
	deviceName << "soDevice" << index;
	device = phScene->FindObject(deviceName.str().c_str())->Cast();
	if (!device) {
		device = fwScene->GetPHScene()->CreateSolid();
	}
	device->SetDynamical(false);
	device->SetGravity(false);

	PHSliderJointDesc sjDesc;
	sjDesc.poseSocket.Ori().RotationArc(Vec3d(0, 0, 1), direction);
	slider = phScene->CreateJoint(device, tool, sjDesc)->Cast();
	slider->SetSpring(5000);	//	5N/mm = 5000N/m 
	slider->SetDamper(50);
	slider->SetTargetPosition(0);
}
