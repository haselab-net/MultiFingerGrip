#ifndef FINGER_GRIP_H
#define FINGER_GRIP_H
#include <Springhead.h>

using namespace Spr;
using namespace std;

struct FingerDesc {
	Vec3d direction;
	Vec3d position;
	double mass = 1;
	double length = 0;
	double maxLength = 1;
};

class Finger: public FingerDesc {
	double force = 0;
public:
	FingerDesc* GetDesc() { return this; }
	void SetDesc(FingerDesc* desc) { *(FingerDesc*)this = *desc; }
	void AddForce(double f);
	void Step(double dt);
	void SetMaxLength(double l) {
		maxLength = l;
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

};

class FingerGrip {
	Posed pose;
public:
	std::vector<Finger> fingers;
	void Step(Posed p, double dt);
};


#endif

