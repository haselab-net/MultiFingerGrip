#ifndef FINGER_GRIP_H
#define FINGER_GRIP_H
#include <Springhead.h>

using namespace Spr;
using namespace std;

class Finger {
	Vec3d direction;
	Vec3d position;
	double mass = 1;
	double length = 0;
	double force = 0;
public:
	void AddForce(double f);
	void Step(double dt);
};

class FingerGrip {
	Posed pose;
public:
	std::vector<Finger> fingers;
	void Step(double dt, Posed p);
};


#endif

