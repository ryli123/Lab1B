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

using namespace std;
const double pi = atan(1) * 4;
const double epsilon = 1E-8; // default margin of error

// magnitude for 2D vector
double findMagnitude(double x, double y) {
	return (sqrt(x * x + y * y));
}

// overload; magnitude for 3D vector
double findMagnitude(double x, double y, double z) {
	return (sqrt(x * x + y * y + z * z));
}

// overload; magnitude of displacement between balls
double findMagnitude(double s1x, double s1y, double s2x, double s2y) {
	return (sqrt((s1x - s2x) * (s1x - s2x) + (s1y - s2y) * (s1y - s2y)));
}

// returns whether spheres have collided by comparing displacement to radii
bool spherecollided2d(double sx1, double sy1, double sx2, double sy2, double radius) {
	// note that magnitude is always positive
	return (findMagnitude(sx1, sy1, sx2, sy2) < 2 * radius + epsilon); 
}

// the function for the force exerted by the spring, for one dimension
double F_s(double compression) {
	// spring function found by regression
	//double k = 11900 * compression + 830;
	//double k = 1000000;

	// Calculates in the form F_s = -kx. Compression parameter is in component form.
	return -1 * (11000 * compression * compression - 850 * compression);
}

// returns appropriate time increment
double findtinc(double distance, double v_magnitude, double radius, bool collided) {
	double dt = 0.001;	// default dt
	
	/* Sets up very small dt when in contact to increase resolution of balls' motion,
	 * sets up larger dt if not in contact to skip through other parts of motion. */
	if (collided)
		dt = 1E-6;
	else {
		/* Based on ratio of distance to velocity of ball system, ensuring that
		 * dt is around 1/10 of the required time for the balls to contact each other. */
		dt = (distance - 2 * radius + epsilon) / v_magnitude / 10;
	}

	return dt;
}

// finds angle between centre of the balls
double findAngle(double s1x, double s1y, double s2x, double s2y) {
	double y = s1y - s2y; //delta y
	double x = s1x - s2x; //delta x
	double angle = 0;

	/* Finds the angle using arctan(y/x). However, it must take in special
	 * cases; when x = 0 arctan does not function, and in other cases arctan
	 * will only give the beta value and must be adjusted by some factor of pi. */
	if (x == 0) {
		if (y > 0)					// 90 deg
			angle = pi / 2;
		else if (y < 0)				// 270 deg
			angle = 3 * pi / 2;
	} else {
		if (x < 0)					// quadrants 2 & 3
			angle = atan(y / x) + pi;
		else if (x > 0 && y < 0)	// quadrant 4
			angle = atan(y / x) + 2 * pi;
		else if (x > 0 && y > 0)	// quadrant 1
			angle = atan(y / x);
	}
	return angle;
}

// truncates value to 4 decimal places
double truncate(double x) {
	return round(x * 10000) / 10000;
}

// with a spring, using just basic kinematics
void springcollision() {
	////sample 1D
	double m1 = 2, m2 = 6;
	double radius = 1;
	double s1x = 0, s1y = 0;
	double s2x = 5, s2y = 0;
	double v1x = 12, v1y = 0;
	double v2x = 0, v2y = 0;
	////Expect v1fx = -6, v2fx as 6 

	//sample 2D static target 
	/*double m1 = 2, m2 = 2;
	double radius = 1;
	double s1x = 0, s1y = 0;
	double s2x = 3, s2y = 1.732;
	double v1x = 2.2, v1y = 0;
	double v2x = 0, v2y = 0; */
	//Expect v1f to be 1.9, theta = -30, v2fx as 1.1, theta = 60
	

	//sample 2D total
	/*double m1 = 2, m2 = 2;
	double radius = 1;
	double s1x = -3.83, s1y = 3.214;
	double s2x = -3.83, s2y = -3.214;
	double v1x = 3.064, v1y = -2.571;
	double v2x = 3.064, v2y = 2.571;*/
	//expect symmetry on final velocities
	
	double compression = 0;
	double angle1 = 0, angle2 = 0;

	double tinc = 0.001;	// default time increment
	double t = 0, printinc = 10;	// prints when printinc = 10;
	double tcollision = 3000;
	bool collision = false;

	/* Set up .csv file to be viewed in excel and allow
	 * data set to be manipulated and made into graphs. */
	ofstream table;
	table.open("springcollision.csv");
	table << "t,sx1,sy1,sx2,sy2,vx1,vy1,vx2,vy2,distance," 
		<< "compression, compression_x, compression_y, F1x, F1y, F2x, F2y\n";

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

			// solve for compression with direction relative to fiducials
			compression = (findMagnitude(s1xc, s1yc, s2xc, s2yc) - 2 * radius) / 2;
			angle1 = findAngle(s1x, s1y, s2x, s2y);
			angle2 = findAngle(s2x, s2y, s1x, s1y);

			s1x += 0.5 * F_s(compression * cos(angle1)) / m1 * tinc * tinc;
			s1y += 0.5 * F_s(compression * sin(angle1)) / m1 * tinc * tinc;
			s2x += 0.5 * F_s(compression * cos(angle2)) / m2 * tinc * tinc;
			s2y += 0.5 * F_s(compression * sin(angle2)) / m2 * tinc * tinc;

			// assuming that both balls are compressed the same amount
			compression = (findMagnitude(s1x, s1y, s2x, s2y) - 2 * radius) / 2;
			v1x += tinc * F_s(compression * cos(angle1)) / m1;
			v1y += tinc * F_s(compression * sin(angle1)) / m1;
			v2x += tinc * F_s(compression * cos(angle2)) / m2;
			v2y += tinc * F_s(compression * sin(angle2)) / m2;
		}

		s1x += v1x * tinc;
		s1y += v1y * tinc;
		s2x += v2x * tinc;
		s2y += v2y * tinc;

		if (printinc == 10) {
			table << truncate(t) << ",";	// t
			table << truncate(s1x) << ","	// displacement
				<< truncate(s1y) << ","
				<< truncate(s2x) << ","
				<< truncate(s2y) << ",";
			table << truncate(v1x) << ","	// velocity
				<< truncate(v1y) << ","
				<< truncate(v2x) << ","
				<< truncate(v2y) << ",";
			table << truncate(findMagnitude(s1x, s1y, s2x, s2y)) << ",";	// distance
			table << truncate(compression) << ","										// compression/force
				<< truncate(compression * cos(angle1)) << ","
				<< truncate(compression * sin(angle1)) << ","
				<< truncate(F_s(compression * cos(angle1))) << ","
				<< truncate(F_s(compression * sin(angle1))) << ","
				<< truncate(F_s(compression * cos(angle2))) << ","
				<< truncate(F_s(compression * sin(angle2))) << ",";
			table << "\n";

			printinc = 0;
		}

		tinc = findtinc(findMagnitude(s1x, s1y, s2x, s2y), findMagnitude(v1x, v1y, v2x, v2y), radius, spherecollided2d(s1x, s1y, s2x, s2y, radius));
		t += tinc;
		printinc++;

	} while (t < tcollision + 1);

	table.close();
}

int main() {
	springcollision();

	return 0;
}