#include "FingerGrip.h"

void Finger::AddForce(double f) {
	force += f;
}
void Finger::Step(PHSolidIf* soGripTool, double dt) {
	//length += 100 * force * dt;
	length = force;
	force = 0;
	LimitLength();

	Posed pose;
	pose.Pos() = position + length*direction;
	pose.Ori() = deviceOrientation;
	spring->SetSocketPose(pose);
}
void Finger::Build(FWSceneIf* fwScene, PHSolidIf* gripDevice) {
	PHSceneIf* phScene = fwScene->GetPHScene();
	ostringstream toolName;
	toolName << "soTool" << index;
	tool = phScene->FindObject(toolName.str().c_str())->Cast();
	if (!tool) {
		tool = fwScene->GetPHScene()->CreateSolid();
	}
	tool->SetGravity(false);
	tool->SetFramePosition(gripDevice->GetPose() * (position + length*direction));
	tool->GetShape(0)->SetStaticFriction(0.7);
	tool->GetShape(0)->SetDynamicFriction(0.65);

	deviceOrientation.RotationArc(Vec3d(1, 1, 1), direction);
	PHSpringDesc sprDesc;
	sprDesc.poseSocket.Ori() = deviceOrientation;
	sprDesc.poseSocket.Pos() = position + length * direction;
	sprDesc.posePlug.Ori() = deviceOrientation;
	sprDesc.spring = 90 * Vec3d(1, 1, 1);	//	0.5N/mm = 500N/m
	sprDesc.damper = sprDesc.spring * 0.01;
	sprDesc.springOri = sprDesc.spring[0] * 1;
	sprDesc.damperOri = sprDesc.damper[0] * 1;

	spring = phScene->CreateJoint(gripDevice, tool, sprDesc)->Cast();
}



FingerGrip::FingerGrip() {
	fingers.resize(2);
	for (int i = 0; i < fingers.size(); ++i) {
		Finger& finger = fingers[i];
		finger.index = i;
		finger.direction = Quaterniond::Rot(Rad(180), 'z') * Quaterniond::Rot(Rad(180 * i), 'z') * Vec3d(1, 0, 0);
		finger.position = Quaterniond::Rot(Rad(180 * i), 'z') * Vec3d(finger.maxLength, 0, 0);
	}
}
void FingerGrip::Build(FWSceneIf* fwScene) {
	gripDevice = fwScene->GetPHScene()->CreateSolid();
	gripDevice->SetGravity(false);
	gripDevice->SetDynamical(false);
	CDBoxDesc bd;
	bd.boxsize = 0.01 * Vec3d(2, 1, 0.5);
	CDShapeIf* shape = fwScene->GetSdk()->GetPHSdk()->CreateShape(bd);
	gripDevice->AddShape(shape);
	gripDevice->SetFramePosition(Vec3d(0,0.2,0));

	//	build fingers
	for (Finger& finger : fingers) {
		finger.Build(fwScene, gripDevice);
	}

	//	set collision mode and tool pose
	fwScene->GetPHScene()->SetContactMode(gripDevice, PHSceneDesc::MODE_NONE);
}

void FingerGrip::Step(Posed p, double dt) {
	pose = p;
	gripDevice->SetPose(pose);
	gripDevice->SetVelocity(Vec3d(0, 0, 0));
	gripDevice->SetAngularVelocity(Vec3d(0, 0, 0));
	for (Finger& finger: fingers) {
		finger.Step(gripDevice, dt);
	}
}

