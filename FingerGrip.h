#ifndef FINGER_GRIP_H
#define FINGER_GRIP_H
#include <Springhead.h>

using namespace Spr;
using namespace std;

struct FingerDesc {
	Vec3d direction;
	Vec3d position;
	double length = 0;
	double maxLength = 0.05;
	PHSolidIf* tool=NULL;			//	Tool's solid (should have shape)
	PHSpringIf* spring = NULL;	//	slider joint from the device to the tool.
};

class Finger: public FingerDesc {
	double force = 0;
	int index = -1;
	Quaterniond deviceOrientation;
public:
	int GetIndex() { return index; }
	FingerDesc* GetDesc() { return this; }
	void SetDesc(FingerDesc* desc) { *(FingerDesc*)this = *desc; }
	void AddForce(double f);
	void Step(PHSolidIf* soGripTool, double dt);
	void SetMaxLength(double l) {
		maxLength = l;
	}
	void Build(FWSceneIf* fwScene, PHSolidIf* gripTool);

	void SetMaterial(const PHMaterial mat) {
		tool->GetShape(0)->SetMaterial(mat);
	}

protected:
	void LimitLength() {
		if (length > maxLength) {
			length = maxLength;
		}
		else if (length < 0) {
			length = 0;
		}
	}
	friend class FingerGrip;
};

class FingerGrip {
public:
	PHSolidIf* gripDevice = NULL;
	Posed pose;
	std::vector<Finger> fingers;
	void Step(Posed p, double dt);
	FingerGrip();
	void Build(FWSceneIf* fwScene);
	void SetMaterial(const PHMaterial mat) {
		for (auto& f : fingers) {
			f.SetMaterial(mat);
		}
	}
};

#endif