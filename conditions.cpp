#include "conditions.hpp"

// Coulomb
Condition con_coulomb(
    CONDITION_COULOMB,
    0.5,
    0.3,
    0.1
);

// LuGre
Condition con_lugre(
    CONDITION_LUGRE,
    1000.0, 1.0, 0.1,
    0.2, 0.7, 400.0,
    0.1
);