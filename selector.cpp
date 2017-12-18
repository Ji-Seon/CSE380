// selector.cpp
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <grvy.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "simpleODE.hpp"
#include "selector.hpp"

// Convenience timer macros for begin/end functions

#define FUNC_BEGIN_TIME gt.BeginTimer(__func__);
#define FUNC_END_TIMER gt.EndTimer(__func__);

using namespace std;

// Globals
GRVY::GRVY_Timer_Class gt; // GRVY Timer

void selector(input x)
{
ofstream myfile;
myfile.open ("outputrkf45.txt"); // Can change the name of the file your data is saved to

//Initialize the timing library - the global timer will be
// Initialized with this call.

gt.Init("GRVY rk timing");
	
	// SimpleODE parameters
	simple2* simpleSol;
        simple2 path[x.max_iter];
	simple2 IVP = simple2();
	
	// Verification mode SimpleODE parameters
	simple2* simSol;
	simple2 hardpath[x.max_iter];
	simple2 Init = simple2();

	simple6* hardSol;
	simple6 hpath[x.max_iter];
	simple6 hIVP = simple6();

	// Verification mode!
	if((x.verif == 0) && (x.problem == 0)){		
		
		// Runs simple solution
		gt.BeginTimer("simpleODE");
		simpleSol = simpleODE(IVP, x, path);
		gt.EndTimer("simpleODE");
		gt.Finalize();

		// Print performance summary to stdout
		gt.Summarize();

		// Reset timers for next iteration
		gt.Reset();

		// Runs rk4 solution
		gt.BeginTimer("simpleODE Verification mode");
		simSol = gsl_simple_trajectory(Init, x, hardpath);
		gt.EndTimer("simpleODE Verification mode");
		gt.Finalize();
		gt.Summarize();	
		gt.Reset();
		
		// puts results from both solvers in one file
		for(int j=0; j < x.max_iter; j++){
			myfile << simpleSol[j].t << " " << simpleSol[j].y << " " << simSol[j].t << " " << simSol[j].y <<  endl;
		}
	}

	else if((x.problem == 0) && (x.solution == 0)){
		simpleSol = simpleODE(IVP, x, path); 
		for(int j=0; j < x.max_iter; j++){
			myfile << simpleSol[j].t << " " << simpleSol[j].y << endl;
		}
		
	}

	// Links hard problem to gsl's rk solver, inside driver allows options.
	else if (x.problem == 1){

		gt.BeginTimer("rk Solver");
		hardSol = gsl_charged_trajectory(hIVP, x, hpath);
		gt.EndTimer("rk Solver");
		gt.Finalize();
		gt.Summarize();
		gt.Reset();
		for(int j=0; j < x.max_iter; j++){
			myfile
			<< hardSol[j].x << " " 
			<< hardSol[j].y << " " 
			<< hardSol[j].z << " " << endl;
		        //<< hardSol[j].u << " " 
			//<< hardSol[j].v << " " 
			//<< hardSol[j].w << endl;
		}
	}
myfile.close();
}
