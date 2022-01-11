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
	tool = phScene->FindObject("soTool")->Cast();

}
