#include "FingerGrip.h"

void Finger::AddForce(double f) {
	force += f;
}
void Finger::Step(double dt) {
	length += force * dt / mass;
	LimitLength();
}


FingerGrip::FingerGrip() {
	fingers.resize(2);
	for (int i = 0; i < fingers.size(); ++i) {
		fingers[i].index = i;
	}
}
void FingerGrip::Build(FWSceneIf* fwScene) {
	for (auto finger : fingers) {
		finger.Build(fwScene);
	}
}


void FingerGrip::Step(Posed p, double dt) {
	pose = p;
	for (auto finger: fingers) {
		finger.Step(dt);
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
	
	ostringstream deviceName;
	deviceName << "soDevice" << index;
	device = phScene->FindObject(deviceName.str().c_str())->Cast();
	if (!device) {
		device = fwScene->GetPHScene()->CreateSolid();
	}
	device->SetDynamical(false);

	PHSliderJointDesc sjDesc;
	sjDesc.poseSocket.Ori().RotationArc(Vec3d(0, 0, 1), direction);
	sjDesc.poseSocket.Pos() = position;	
	slider = phScene->CreateJoint(device, tool, sjDesc)->Cast();
	slider->SetSpring(1);
	slider->SetDamper(0.1);
	slider->SetTargetPosition(length);
}
