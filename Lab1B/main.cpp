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
const double epsilon = 0.001; // default margin of error

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

// moment of inertia for solid sphere
double ballMoI(double r, double m)
{
	return(2 * m * r * r / 5);
}

// magnitude of cross product of two vectors in 2d
double crossProductMag(double x1, double y1, double x2, double y2)
{
	return(findMagnitude(y1 - y2, -(x1 - x2), x1 * y2 - y1 - x2));
}

// returns whether spheres have collided by comparing displacement to radii
bool spherecollided2d(double sx1, double sy1, double sx2, double sy2, double radius) {
	// note that magnitude is always positive
	return (findMagnitude(sx1, sy1, sx2, sy2) < 2 * radius + epsilon); 
}

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

double findTorque(double Fx, double Fy, double rx, double ry)
{
	double sinT = crossProductMag(Fx, Fy, rx, ry) / (findMagnitude(Fx, Fy) * findMagnitude(rx, ry));
	return findMagnitude(rx, ry) * findMagnitude(Fx, Fy) * sinT;
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

// truncates value to 3 decimal places
double truncate(double x) {
	return round(x * 1000) / 1000;
}

// with a spring, using just basic kinematics
void springcollision() {
	double compression = 0;

	//double m1 = 5, m2 = 2.5;	// kg
	//double radius = 1;	// m
	// sample 1d collision with stationary target; SI units
	double m1 = 2, m2 = 6;
	double radius = 1;
	double s1x = 0, s1y = 0;
	double s2x = 3, s2y = 1;
	double v1x = 4.5, v1y = 0;
	double v2x = 0, v2y = 0;
	double tinc = 0.001;

	//double tinc = 0.001;	// default time increment
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

			s1x += 0.5 * F_s(s1xc-s2xc, compression) / m1 * tinc * tinc;
			s1y += 0.5 * F_s(s1yc-s2yc, compression) / m1 * tinc * tinc;
			s2x += 0.5 * F_s(s2xc-s1xc, compression) / m2 * tinc * tinc;
			s2y += 0.5 * F_s(s2yc-s1yc, compression) / m2 * tinc * tinc;

			// assuming that both balls are compressed the same amount
			compression = (findMagnitude(s1x, s1y, s2x, s2y) - 2 * radius) / 2;
			v1x += tinc * F_s(s1xc-s2xc, compression) / m1;
			v1y += tinc * F_s(s1yc-s2yc, compression) / m1;
			v2x += tinc * F_s(s2xc-s1xc, compression) / m2;
			v2y += tinc * F_s(s2yc-s1yc, compression) / m2;
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

// glancing collision, end rotation, no initial rotation
void glancingcollision() {
	double compression = 0;

	double m1 = 2, m2 = 6;
	double s1x = 0, s1y = 0;
	double s2x = 5, s2y = 0;
	double v1x = 12, v1y = 0;
	double v2x = 0, v2y = 0;
	double radius = 1;
	double I1 = ballMoI(m1, radius);
	double I2 = ballMoI(m2, radius);
	double omega1 = 0, omega2 = 0;
	double tinc = 0.001;

	double t = 0, printinc = 10;
	double tcollision = 3000;
	bool collision = false;
	
	/* Set up .csv file to be viewed in excel and allow
	 * data set to be manipulated and made into graphs. */
	ofstream table;
	table.open("newtable.csv");
	table << "t,sx1,sy1,sx2,sy2,vx1,vy1,vx2,vy2,distance,total KE\n";

	/* Loop runs until a collision occurs and then runs for another second
	 * to display data after collision. */
	do {
		// check if collided
		if (spherecollided2d(s1x, s1y, s2x, s2y, radius)) {
			if (!collision) {
				cout << "\nCollided.\n";
				tcollision = t;
				collision = true;
			}
			// take current position to test direction of spring force
			double s1xc = s1x, s1yc = s1y, s2xc = s2x, s2yc = s2y;

			s1x += 0.5 * F_s(s1xc - s2xc, compression) / m1 * tinc * tinc;
			s1y += 0.5 * F_s(s1yc - s2yc, compression) / m1 * tinc * tinc;
			s2x += 0.5 * F_s(s2xc - s1xc, compression) / m2 * tinc * tinc;
			s2y += 0.5 * F_s(s2yc - s1yc, compression) / m2 * tinc * tinc;

			// assuming that both balls are compressed the same amount
			compression = (findMagnitude(s1x, s1y, s2x, s2y) - 2 * radius) / 2;
			v1x += tinc * F_s(s1xc - s2xc, compression) / m1;
			v1y += tinc * F_s(s1yc - s2yc, compression) / m1;
			v2x += tinc * F_s(s2xc - s1xc, compression) / m2;
			v2y += tinc * F_s(s2yc - s1yc, compression) / m2;

			// note that by conservation of angular momentum, m1 * omega1  + m2 * omega2 is invariant
			omega1 += findTorque(F_s(s1xc - s2xc, compression), F_s(s1yc - s2yc, compression), s1xc, s1yc) / I1 * tinc;
			omega2 += findTorque(F_s(s2xc - s1xc, compression), F_s(s2yc - s1yc, compression), s2xc, s2yc) / I2 * tinc;


		}

		s1x += v1x * tinc;
		s1y += v1y * tinc;
		s2x += v2x * tinc;
		s2y += v2y * tinc;

		if (printinc == 10) {
			table	<< round(t * 1000) / 1000 << ",";	// t
			table	<< round(s1x * 1000) / 1000 << ","	// displacement
					<< round(s1y * 1000) / 1000 << "," 
					<< round(s2x * 1000) / 1000 << "," 
					<< round(s2y * 1000) / 1000 << ",";
			table	<< round(v1x * 1000) / 1000 << ","	// velocity
					<< round(v1y * 1000) / 1000 << "," 
					<< round(v2x * 1000) / 1000 << "," 
					<< round(v2y * 1000) / 1000 << ",";
			table	<< round(findMagnitude(s1x, s1y, s2x, s2y) * 1000) / 1000 << ",";	// distance
			table	<< round(0.5 * (m1 * sqrt(v1x * v1x + v1y * v1y)	// kinetic energy
					+ m2 * sqrt(v2x * v2x + v2y * v2y) 
					+ 0.4 * radius * radius * (m1 * omega1 * omega1 + m2 * omega2 * omega2)) * 1000) / 1000 << ",";
			table	<< "\n";
			printinc = 0; //reset increment to 0
		}

		t += tinc;
		printinc++;
	} while (t < tcollision + 1);

	table.close();
}

int main() {
	springcollision();
	//glancingcollision();

	return 0;
}