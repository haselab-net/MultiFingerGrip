#include "FingerGrip.h"

void Finger::AddForce(double f) {
	force += f;
}
void Finger::Step(PHSolidIf* soGripTool, double dt) {
	length += force * dt / mass;
	force = 0;
	LimitLength();

	//	Posed pose = soGripTool->GetPose() * Posed::Trn(position + length*direction);
	Posed pose;
	pose.Pos() = position + length*direction;
	pose.Ori() = deviceOrientation;
	slider->SetSocketPose(pose);
}
void Finger::Build(FWSceneIf* fwScene, PHSolidIf* gripTool) {
	PHSceneIf* phScene = fwScene->GetPHScene();
	ostringstream toolName;
	toolName << "soTool" << index;
	tool = phScene->FindObject(toolName.str().c_str())->Cast();
	if (!tool) {
		tool = fwScene->GetPHScene()->CreateSolid();
	}
	tool->SetGravity(false);

	deviceOrientation.RotationArc(Vec3d(0, 0, 1), direction);
	PHSliderJointDesc sjDesc;
	sjDesc.poseSocket.Ori() = deviceOrientation;
	sjDesc.poseSocket.Pos() = position + length * direction;
	slider = phScene->CreateJoint(gripTool, tool, sjDesc)->Cast();
	slider->SetSpring(5000);	//	5N/mm = 5000N/m 
	slider->SetDamper(slider->GetSpring() * 0.1);
	slider->SetTargetPosition(0);
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
	gripTool = fwScene->GetPHScene()->CreateSolid();
	gripTool->SetGravity(false);
	CDBoxDesc bd;
	bd.boxsize = 0.01 * Vec3d(1, 2, 0.5);
	CDShapeIf* shape = fwScene->GetSdk()->GetPHSdk()->CreateShape(bd);
	shape->SetDensity(100);
	gripTool->AddShape(shape);
	gripTool->CompInertia();
	gripTool->SetDynamical(true);

	gripDevice = fwScene->GetPHScene()->CreateSolid();
	gripDevice->SetGravity(false);
	gripDevice->SetDynamical(false);
	bd.boxsize = 0.01 * Vec3d(2, 1, 0.5);
	shape = fwScene->GetSdk()->GetPHSdk()->CreateShape(bd);
	shape->SetDensity(0.1f);
	gripDevice->AddShape(shape);
	gripDevice->CompInertia();


	PHSpringDesc sprd;
	sprd.spring = Vec3f(1, 1, 1) * 5000;	//	5N/mm
	sprd.damper = sprd.spring * 0.1;		//	small damper
	sprd.springOri = sprd.spring.x;
	sprd.damperOri = sprd.damper.x;
	spring = fwScene->GetPHScene()->CreateJoint(gripTool, gripDevice, sprd)->Cast();

	//	build fingers
	for (Finger& finger : fingers) {
		finger.Build(fwScene, gripTool);
	}

	Vec3d gripPosition = Vec3d(0, 0.2, 0);
	Step(Posed::Trn(gripPosition), 0.01);	//	set device and finger position
	gripTool->SetFramePosition(gripPosition);	//	set tool position

	//	set collision mode and tool pose
	fwScene->GetPHScene()->SetContactMode(gripTool, PHSceneDesc::MODE_NONE);
	fwScene->GetPHScene()->SetContactMode(gripDevice, PHSceneDesc::MODE_NONE);
}

void FingerGrip::Step(Posed p, double dt) {
	pose = p;
	gripDevice->SetPose(pose);
	gripDevice->SetVelocity(Vec3d(0, 0, 0));
	gripDevice->SetAngularVelocity(Vec3d(0, 0, 0));
	for (Finger& finger: fingers) {
		finger.Step(gripTool, dt);
	}

	Vec6d f = spring->GetMotorForce();
	DSTR << "Grip force: " << f << "ori:" << gripDevice->GetOrientation() << gripTool->GetOrientation() << std::endl;
}

