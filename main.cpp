// main.cpp
// Driver for the programs
#include <iostream>
#include <cstdlib>
#include <string>
#include <stdexcept>
#include <vector>

#include "parse.hpp"
#include "class_def.hpp"
#include "selector.hpp"



int main(int argc, char** argv){
	
	input read;

	// reads input file "input.dat" and returns an object "read" containg parameters problem, solMethod, etc...
	try{
	read = parse(argc, argv);
	//cout << endl << endl;
	// cout << read.problem << endl;
	selector(read);
	}
	// throws a fit if reading attempt didn't work
	catch(exception& e){
		cerr << "An error occured during read attempt" << endl;
		cerr << "Error: " << e.what() << endl;
	return 1;
	}
	

return 0;
}
