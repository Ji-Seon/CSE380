// simpleODE.hpp
#ifndef simpleODE_H
#define simpleODE_H

#include "class_def.hpp"

using namespace std;


double AnalyticSol(double t);
simple2* simpleODE(simple2 state, input data, simple2 path[]);
int simple_func(double t, const double y[], double f[], void *params);
int simple_jac(double t, const double y[], double *dfdy, double dfdt[], void *params);

simple2* gsl_charged_trajectory(simple2 initState, input opts, simple2 path[]);
simple2* gsl_simple_trajectory(simple2 initState, input opts, simple2 path[]);

#endif
