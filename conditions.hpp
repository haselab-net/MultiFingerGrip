#define CONDITION_LUGRE 1
#define CONDITION_COULOMB 0

#pragma once

typedef struct Condition {
    unsigned int friction_model;
    union {
        struct {
            double sigma0;
            double sigma1;
            double sigma2;
            double A;
            double B;
            double C;
        } lugre;
        struct {
            double mu0;
            double mu;
        } coulomb;
    };
    double mass;

	// Default
	Condition()
		: friction_model(CONDITION_COULOMB), mass(0.1)
	{
		lugre = {};
		coulomb = { 0.5, 0.3 };
	}

    // Coulomb
    Condition(unsigned int model, double mu0, double mu, double m)
        : friction_model(model), mass(m)
    {
        lugre = {};
        coulomb = { mu0, mu };
    }

    // LuGre
    Condition(unsigned int model,
        double s0, double s1, double s2,
        double A_, double B_, double C_,
        double m)
        : friction_model(model), mass(m)
    {
        lugre = { s0, s1, s2, A_, B_, C_ };
    }
};

/*
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
    1000.0, 50.0, 0.1,
    0.2, 0.7, 400.0,
    0.1
);
*/

extern Condition con_coulomb;
extern Condition con_lugre;