#include "FingerGrip.h"

void Finger::AddForce(double f) {
	force += f;
}
void Finger::Step(PHSolidIf* soGripTool, double dt) {
	//length += force * dt;
	length = force / 6;
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
	
	CDSphereDesc shapeDesc;
	shapeDesc.radius = 0.005f;
	shapeDesc.material.mu = shapeDesc.material.mu0 = 1.0;
	shapeDesc.material.rotationFriction = 0.0f;
	shapeDesc.material.frictionModel = 1;
	shapeDesc.material.bristlesSpringK = 4000.0;	//	1000N/m
	shapeDesc.material.bristlesDamperD = 5.0;	//	0.1Ns/m
	shapeDesc.material.bristlesViscosityV = 0.6;	//	0.1Ns/m
	shapeDesc.material.timeVaryFrictionA = .2;
	shapeDesc.material.timeVaryFrictionB = .6;
	shapeDesc.material.timeVaryFrictionC = 400.0;
	//shapeDesc.material.frictionModel = FrictionModel::COULOMB;
	//tool = fwScene->GetPHScene()->CreateSolid();
	CDShapeIf* sh = fwScene->GetSdk()->GetPHSdk()->CreateShape(shapeDesc);
	sh->SetDensity(0.0006f / sh->CalcVolume());
	tool->RemoveShape(0);
	tool->AddShape(sh);
	
	tool->SetGravity(false);
	tool->SetFramePosition(gripDevice->GetPose() * (position + length*direction));

	deviceOrientation.RotationArc(Vec3d(1, 0, 0), direction);
	PHSpringDesc sprDesc;
	sprDesc.poseSocket.Ori() = deviceOrientation;
	sprDesc.poseSocket.Pos() = position + length * direction;
	sprDesc.posePlug.Ori() = deviceOrientation;
	sprDesc.spring = 500.0 * Vec3d(1, 1, 1);	//	0.5N/mm = 500N/m
	sprDesc.damper = sprDesc.spring * 0.01;
	sprDesc.springOri = sprDesc.spring[0];
	sprDesc.damperOri = sprDesc.damper[0];
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

