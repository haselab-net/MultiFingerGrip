#include <Springhead.h>

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
    double mass0; // Initial Mass
	double dmdt; // Mass increment rate [kg/s]

	// Default
	Condition()
		: friction_model(Spr::FrictionModel::COULOMB), mass0(0.1), dmdt(2.0/9.8)
	{
		lugre = {};
		coulomb = { 0.5, 0.3 };
	}

    // Coulomb
    Condition(unsigned int model, double mu0, double mu, double m, double dmdt)
        : friction_model(model), mass0(m), dmdt(2.0/9.8)
    {
        lugre = {};
        coulomb = { mu0, mu };
    }

    // LuGre
    Condition(unsigned int model,
        double s0, double s1, double s2,
        double A_, double B_, double C_,
        double m, double  dmdt)
		: friction_model(model), mass0(m), dmdt(2.0 / 9.8)
    {
        lugre = { s0, s1, s2, A_, B_, C_ };
    }
};


extern Condition con_coulomb;
extern Condition con_lugre;