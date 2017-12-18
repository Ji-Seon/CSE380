#include <iostream>
#include <cmath>
#include <stdexcept>
#include <cassert>
#include <cstring>
#include <vector>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "hardODE.hpp"
#include "class_def.hpp"

using namespace std;

/* Charged particle system:
 * parameters: omega, tau
 *    x' = u
 *    y' = v
 *    z' = w
 *    u' = omega * v - u / tau
 *    v' = -omega * u - v / tau
 *    w' = -w / tau
*/

/* Mapping of variables into arrays:
 *    x = y[0]          x' = f[0]
 *    y = y[1]          y' = f[1]
 *    z = y[2]          z' = f[2]
 *    u = y[3]          u' = f[3]
 *    v = y[4]          v' = f[4]
 *    w = y[5]          w' = f[5]
*/


// Code adopted from gsl tutorial: https://www.gnu.org/software/gsl/manual/html_node/Stepping-Functions.html

// solves the trajectory x(t) (acceleration)
int charged_solver(double t, const double y[], double f[],void *params)
{

	double* dParams = static_cast<double*>(params);
  	double omega = dParams[0];
  	double tau = dParams[1];
	
  	f[0] = y[3];
  	f[1] = y[4];
  	f[2] = y[5];
  	f[3] = omega * y[4] - y[3] / tau;
  	f[4] = -omega * y[3] - y[4] / tau;
  	f[5] = -y[5] / tau;

  	return GSL_SUCCESS;
}


// GSL function to calculate the jacobian
int charged_jac(double t, const double y[], double *dfdy,double dfdt[], void *params)
{

  	double* dParams = static_cast<double*>(params);
  	double omega = dParams[0];
  	double tau = dParams[1];
  	gsl_matrix_view dfdy_mat = gsl_matrix_view_array (dfdy, 6, 6);
	gsl_matrix * m = &dfdy_mat.matrix;
  
/* To avoid explicitly zeroing lots and lots of things, zero the matrix
 * first and only explcitly set nonzero elements. */
  	gsl_matrix_set_zero(m);

  	gsl_matrix_set (m, 0, 3, 1.0); // dx'/du = 1, all others 0
  	gsl_matrix_set (m, 1, 4, 1.0); // dy'/dv = 1, all others 0
  	gsl_matrix_set (m, 2, 5, 1.0); // dz'/dw = 1, all others 0

// Set matrix entries for u' 
  	gsl_matrix_set (m, 3, 3, - (1/tau));  // du'/du = -1 / tau
  	gsl_matrix_set (m, 3, 4, omega);      //du'/dv = omega

// Set matrix entries for v'
  	gsl_matrix_set (m, 4, 3, -omega);     //dv'/du = -omega
  	gsl_matrix_set (m, 4, 4, -(1/tau));   //dv'/dv = -1/tau
  
// Only one matrix entry for w'
  	gsl_matrix_set (m, 5, 5, -(1/tau));     //dw'/dw = -1/tau

// None of the derivatives have explicit dependence on t
  	memset(dfdt, 6, sizeof(double));
  
  	return GSL_SUCCESS;
}


// Main driver
simple6*  gsl_charged_trajectory(simple6 initState, input opts, simple6 path[])
{

// Initialize the parts of GSL ODE that relate only to the problem.

// "You can assume omega and tau are both set to the value of 5"
	double params[2] = {5.0,5.0};
  	gsl_odeiv2_system sys = {charged_solver, charged_jac, 6, params};

// Read in options and initial state
  	int nsteps = opts.max_iter;
  	double h = opts.stepSize;
  
// Initialize the parts that rely on input
  	double y[6];
  	y[0] = initState.x;
  	y[1] = initState.y;
  	y[2] = initState.z;
  	y[3] = initState.u;
  	y[4] = initState.v;
  	y[5] = initState.w;

// Driver requires several variables that we won't really use.
  	const double dummyHStart = h;
  	const double dummyEpsAbs = 1e100;
  	const double dummyEpsRel = 1e100;


	gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(&sys,gsl_odeiv2_step_rk4,dummyHStart,dummyEpsAbs,dummyEpsRel);
	
	if(opts.solution == 1){
      		gsl_odeiv2_driver_free(d);
		gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(&sys,gsl_odeiv2_step_rk4,dummyHStart,dummyEpsAbs,dummyEpsRel);
	cout << "You have chosen rk4" << endl;
	}
	
	else if(opts.solution == 2){
		gsl_odeiv2_driver_free(d);
      		gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(&sys,gsl_odeiv2_step_rkck,dummyHStart,dummyEpsAbs,dummyEpsRel);
	cout << "You have chosen rkck" << endl;
	}
	
	else if(opts.solution == 3){
		gsl_odeiv2_driver_free(d);
      		gsl_odeiv2_driver * d= gsl_odeiv2_driver_alloc_y_new(&sys,gsl_odeiv2_step_rkf45,dummyHStart,dummyEpsAbs,dummyEpsRel);
	cout << "You have chosen rkf45" << endl;
	}
	
  // Time is not explicitly used for results
double t = 0.0;
simple6 state;
	
for (int i = 0; i < nsteps; i++)
{ 
      // Runs a single-step iteration of the ODE
	int status = gsl_odeiv2_driver_apply_fixed_step(d, &t, h, 1, y);

	if (status != GSL_SUCCESS){
        cout << gsl_strerror(status) << endl;
        throw std::runtime_error(gsl_strerror(status));
       }



        state.x = y[0];
	state.y = y[1];
	state.z = y[2];
	state.u = y[3];
	state.v = y[4];
	state.w = y[5];
	path[i] = state;
}

gsl_odeiv2_driver_free(d);

return path;
}

