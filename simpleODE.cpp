// Solve first order ODE of my choice using forward euler method
// Implementation file


// Problem is: y^t + 2y = 2-e^-4t
// Re-written as: y^t = 2 - e^-4t -2y
// With t0=0, y0=1
// Analytical Solution: y(t) = 1 + (1/2)exp(-4t) - (1/2)e^(-2t)

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "simpleODE.hpp"
#include "class_def.hpp"

using namespace std;

// Calculates one iteration/time step using forward euler
simple2* simpleODE(simple2 state, input data, simple2 path[]){

	
	for (int j=0; j<data.max_iter; j++){
        // m = temp f(t,y) solution
        // t0,y0 used to solve problem iteratively
        // t1,y1 temp updated t0, t1 values
        double y = state.y;
	double t = state.t;
	
	// Forward Euler calculations
	double dt = data.stepSize;
	double dy = 2-exp(-4*t)-(2*y);

	// incrementing solution values
	t += dt;
	y += (dt*dy);
	
	// updating state of solution
	state.y = y;
	state.t = t;
	path[j] = state;
	}

	return path;	

}



// Obvi the Analytical solution calculation
double AnalyticSol(double t)
{
	return (1 + ((1/2)*exp(-4*t))-((1/2)*exp(-2*t)));
}

int simple_func (double t, const double y[], double f[], void *params)
{
(void)y;
double* dParams = static_cast<double*>(params);
double m = dParams[0];
double n = dParams[1];
f[0] = 1 + ((1/2)*exp(-4*t))-((1/2)*exp(-2*t));
return GSL_SUCCESS;
}

/* Jabobian function for GSL ODE module */
int simple_jac (double t, const double y[], double *dfdy,double dfdt[], void *params)
{
double* dParams = static_cast<double*>(params);
double m = dParams[0];
double n = dParams[1];
gsl_matrix_view dfdy_mat = gsl_matrix_view_array (dfdy, 1, 1);
gsl_matrix * matt = &dfdy_mat.matrix;
gsl_matrix_set (matt, 0, 0, 0.0);
dfdt[0] = 1 + ((1/2)*exp(-4*t))-((1/2)*exp(-2*t));
return GSL_SUCCESS;
}

simple2* gsl_simple_trajectory(simple2 initState, input opts, simple2 path[])
{

// Initialize the parts of GSL ODE that relate only to the problem.
  
/* In the general form of the problem, there are two parameters. In this case,
 * I've specialized them to 4 and 2, so I can record them in the code itself*/
double params[2] = {0,1};
gsl_odeiv2_system sys = {simple_func, simple_jac, 1, params};

// Read in options and initial state
double t = initState.t;
double y = initState.y;
int nsteps = opts.max_iter;
double h = opts.stepSize;

// Initialize the parts that rely on input
double c[2];
c[1] = y; 

  
// Driver requires several variables that we won't really use.
const double dummyHStart = h;
const double dummyEpsAbs = 1e100;
const double dummyEpsRel = 1e100;
simple2 state;

gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new (&sys, gsl_odeiv2_step_rk4, dummyHStart, dummyEpsAbs, dummyEpsRel);

for (int i = 0; i < nsteps; i++)
    { 
      // Runs a single-step iteration of the ODE
      int status = gsl_odeiv2_driver_apply_fixed_step (d, &t, h, 1, c);

      if (status != GSL_SUCCESS){
       cout << gsl_strerror(status) << endl;
       throw std::runtime_error(gsl_strerror(status));
      }
    state.y = c[0];
    state.t = c[1];
    path[i] = state;    
    }

  gsl_odeiv2_driver_free (d);
  return path;
}

