#include <Springhead.h>
#include "conditions.hpp"

using namespace Spr;

// Coulomb
Condition con_coulomb(
    FrictionModel::COULOMB,
    0.9,
    0.2,
    0.5,
	4.0 / 9.8
);

// LuGre
Condition con_lugre(
    FrictionModel::LUGRE,
    1000.0, 0.01, 1.0,
    0.2, 0.7, 0.001,
    0.5,
	4.0 / 9.8
);