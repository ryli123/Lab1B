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
#include "Windows.h"
#include <math.h>

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
	// absval and sign of compression
	double compressAbs = fabs(compression);
	int sign = compression / compressAbs;

	/* Function based on experiment, which gives the function F = -(6730+-20)x^(3/2). 
	 * Compression parameter is in component form. Can only take the sqrt of the abs val
	 * of the compression as sqrt(-1) is not allowed; sign adjusts accordingly. */
	return -6730 * sign * sqrt(pow(compressAbs,3));
}

// returns appropriate time increment
double findtinc(double distance, double v_magnitude, double radius, bool collided) {
	double dt = 0.001;	// default dt
	
	/* Sets up very small dt when in contact to increase resolution of balls' motion,
	 * sets up larger dt if not in contact to skip through other parts of motion. */
	if (collided)
		dt = 1E-5;
	else {
		/* Based on ratio of distance to velocity of ball system, ensuring that
		 * dt is around 1/10 of the required time for the balls to contact each other. 
		 * Lower bound set at 0.001. */
		dt = (distance - 2 * radius + epsilon) / v_magnitude / 50;
		if (dt < 1E-3) 
			dt = 1E-3;
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

// overload; finds the angle of velocity
double findAngle(double vx, double vy) {
	double y = vy; //delta y
	double x = vx; //delta x
	double angle = 0;

	if (x == 0) {
		if (y > 0)					// 90 deg
			angle = pi / 2;
		else if (y < 0)				// 270 deg
			angle = 3 * pi / 2;
	}
	else {
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
int main() {
	////sample 1D
	/*double m1 = 2, m2 = 6;
	double radius = 1;
	double s1x = 0, s1y = 0;
	double s2x = 5, s2y = 0;
	double v1x = 12, v1y = 0;
	double v2x = 0, v2y = 0;*/
	////Expect v1fx = -6, v2fx as 6

	//sample 2D static target
	double m1 = 0.0198, m2 = 0.0198;
	double radius = 0.0307;
	double s1x = 0, s1y = 0;
	double s2x = 3, s2y = 0.0518;
	double v1x = 2.2, v1y = 0;
	double v2x = 0, v2y = 0;
	//Expect v1f to be 1.9, theta = -30, v2fx as 1.1, theta = 60
	// 1.645, -0.95, 0.55, 0.95
	
	////sample 2D total
	/*double m1 = 0.0198, m2 = 0.0198;
	double radius = 0.0307;
	double s1x = -3.83, s1y = 3.214;
	double s2x = -3.83, s2y = -3.214;
	double v1x = 3.064, v1y = -2.571;
	double v2x = 3.064, v2y = 2.571;*/
	////expect symmetry on final velocities
	//// v1f 4, 40 deg, v2f 4, -40 deg
	
	double compression = 0;
	double angle1 = 0, angle2 = 0;

	double tinc = 0.001;	// default time increment
	double t = 0, printinc = 40;	// prints when printinc = 40;
	double tcollision = 500;
	bool collision = false;

	/* Set up .csv file to be viewed in excel and allow
	 * data set to be manipulated and made into graphs. */
	ofstream table;
	table.open("table.csv");
	table << "t (s),sx1 (m),sy1 (m),sx2 (m),sy2 (m),vx1 (ms^-1),vy1 (ms^-1),"
			<< "v1 (ms^-1),vAngle1 (deg),vx2 (ms^-1),vy2 (ms^-1),v2 (ms^-1),"
			<< "vAngle2 (deg),distance (m),\n";

	/* This loop runs until the default time limit (tcollision + 1) ends
	 * or until one second passes since the collision, allowing for some time
	 * for the balls to collide and compress, noting the exit velocities. */
	do {
		// check if collided
		if (spherecollided2d(s1x, s1y, s2x, s2y, radius)) {
			// sets collision time once
			if (!collision) {
				cout << "\nCollided.\n";
				tcollision = t;
				collision = true;
			}

			// solve for compression with direction relative to fiducials
			compression = (findMagnitude(s1x, s1y, s2x, s2y) - 2 * radius) / 2;
			angle1 = findAngle(s1x, s1y, s2x, s2y);		// ang1 and ang2 will be
			angle2 = findAngle(s2x, s2y, s1x, s1y);		// 180 deg apart

			// change positions by factor of 1/2 * at^2
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

		// adjust positions by velocity travelled in the dt
		s1x += v1x * tinc;
		s1y += v1y * tinc;
		s2x += v2x * tinc;
		s2y += v2y * tinc;

		// prints out all the required data onto the csv file
		if (printinc == 40) {
			table	<< truncate(t) << ","	// t
					<< truncate(s1x) << ","	// displacement
					<< truncate(s1y) << ","
					<< truncate(s2x) << ","
					<< truncate(s2y) << ","
					<< truncate(v1x) << ","	// velocity (cartesian and polar)
					<< truncate(v1y) << ","
					<< truncate(findMagnitude(v1x, v1y)) << ","
					<< truncate(findAngle(v1x, v1y) * 180 / pi) << ","
					<< truncate(v2x) << ","
					<< truncate(v2y) << ","
					<< truncate(findMagnitude(v2x, v2y)) << ","
					<< truncate(findAngle(v2x, v2y) * 180 / pi) << ","
					<< truncate(findMagnitude(s1x, s1y, s2x, s2y)) << ","//dist
					<< "\n";

			printinc = 0;
		}

		// set appropriate increment and change time
		tinc = findtinc(findMagnitude(s1x, s1y, s2x, s2y), 
						findMagnitude(v1x, v1y, v2x, v2y), 
						radius, 
						spherecollided2d(s1x, s1y, s2x, s2y, radius));
		t += tinc;
		printinc++;

	} while (t < tcollision + 1);	// ends when 1 second passed since collision

	table.close();

	return 0;
}