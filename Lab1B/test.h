#pragma once

/*
 * Ryan Li, Justin Ye, Simhon Chourasia, Eric Zhao, Austin Lin
 * Lab 1A - Collisions Simulator
 * SPH4U0
 *
 * Simulates the collision of two circular objects as an elastic
 * collision with spring forces regressed by experiment.
 */

#include <iostream>
#include <cmath>
#include <fstream>
#include "header.h"
#include "Windows.h"
#include <math.h>

#ifndef RUN
#endif

// the function for the force exerted by the spring, for one dimension
double F_s(double displacement, double compression) {
	// spring function found by regression
	//double k = 11900 * compression + 830;

	double k = 1000000;

	/* Solves for spring force acting on ball using F_s = -kx.
	 * Solves for one desired component only, and displacement
	 * is given for the component. No force if displacement 0;
	 * otherwise, sign of the displacement gives direction of force. */
	if (displacement == 0)
		return 0;
	else {
		int direction = displacement / fabs(displacement);	// gives sign
		return -1 * direction * k * compression;
	}
}

// with a spring, using just basic kinematics
void springcollision() {
	////sample 1D
	//double m1 = 2, m2 = 6;
	//double radius = 1;
	//double s1x = 0, s1y = 0;
	//double s2x = 5, s2y = 0;
	//double v1x = 12, v1y = 0;
	//double v2x = 0, v2y = 0;
	////Expect v1fx = -6, v2fx as 6 

	//sample 2D static target
	double m1 = 2, m2 = 2;
	double radius = 1;
	double s1x = 0, s1y = 0;
	double s2x = 3, s2y = 1.732;
	double v1x = 2.2, v1y = 0;
	double v2x = 0, v2y = 0;
	//Expect v1f to be 1.9, theta = -30, v2fx as 1.1, theta = 60


	//sample 2D total
	/*double m1 = 2, m2 = 2;
	double radius = 1;
	double s1x = -3.83, s1y = 3.214;
	double s2x = -3.83, s2y = -3.214;
	double v1x = 3.064, v1y = -2.571;
	double v2x = 3.064, v2y = 2.571;*/
	//expect symmetry on final velocities

	double compress = 0;
	double compressAngle = 0;
	double tinc = 0.001;	// default time increment
	double t = 0, printinc = 10;	// prints when printinc = 10;
	double tcollision = 3000;
	bool collision = false;

	/* Set up .csv file to be viewed in excel and allow
	 * data set to be manipulated and made into graphs. */
	ofstream table;
	table.open("springcollision.csv");
	table << "t,sx1,sy1,sx2,sy2,vx1,vy1,vx2,vy2,distance\n";

	do {
		// check if collided
		if (spherecollided2d(s1x, s1y, s2x, s2y, radius)) {
			// sets collision time once
			if (!collision) {
				cout << "\nCollided.\n";
				tcollision = t;
				collision = true;
			}

			// take current position to test direction of spring force
			double s1xc = s1x, s1yc = s1y, s2xc = s2x, s2yc = s2y;

			s1x += 0.5 * F_s(s1xc - s2xc, compressionx) / m1 * tinc * tinc;
			s1y += 0.5 * F_s(s1yc - s2yc, compressiony) / m1 * tinc * tinc;
			s2x += 0.5 * F_s(s2xc - s1xc, compressionx) / m2 * tinc * tinc;
			s2y += 0.5 * F_s(s2yc - s1yc, compressiony) / m2 * tinc * tinc;

			// assuming that both balls are compressed the same amount
			compress = (findMagnitude(s1xc, s1yc, s2xc, s2yc) - 2 * radius) / 2;
			v1x += tinc * F_s(s1xc - s2xc, compressionx) / m1;
			v1y += tinc * F_s(s1yc - s2yc, compressiony) / m1;
			v2x += tinc * F_s(s2xc - s1xc, compressionx) / m2;
			v2y += tinc * F_s(s2yc - s1yc, compressiony) / m2;
		}

		s1x += v1x * tinc;
		s1y += v1y * tinc;
		s2x += v2x * tinc;
		s2y += v2y * tinc;

		if (printinc == 10) {
			table << truncate(t) << ",";		// t
			table << truncate(s1x) << ","	// displacement
				<< truncate(s1y) << ","
				<< truncate(s2x) << ","
				<< truncate(s2y) << ",";
			table << truncate(v1x) << ","	// velocity
				<< truncate(v1y) << ","
				<< truncate(v2x) << ","
				<< truncate(v2y) << ",";
			table << truncate(findMagnitude(s1x, s1y, s2x, s2y)) << ",";	// distance
			table << "\n";

			printinc = 0;
		}

		tinc = findtinc(findMagnitude(s1x, s1y, s2x, s2y), findMagnitude(v1x, v1y, v2x, v2y), radius, spherecollided2d(s1x, s1y, s2x, s2y, radius));
		t += tinc;
		printinc++;

	} while (t < tcollision + 1);

	table.close();
}