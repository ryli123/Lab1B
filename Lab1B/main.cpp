/*
 * Ryan Li, Justin Ye, Simhon Chourasia, Eric Zhao, Austin Lin
 * Lab 1B - Collisions
 * 15 Oct 19
 *
 * Simulates the collision of two spherical objects and computes
 * position/rotation over time.
 */

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <fstream>
#include "header.h"

#ifndef RUN
#endif

#include "Windows.h"
#include <math.h>

using namespace std;
constexpr double pi = M_PI;	// renaming M_PI constant to pi (3.14...)
const double epsilon = 0.001; // margin of error

//magnitude for single vector
double findMagnitude(double x, double y)
{
	return (sqrt(x * x + y * y));
}

//magnitude for 3D vector
double findMagnitude(double x, double y, double z)
{
	return (sqrt(x * x + y * y + z*z));
}

double findMagnitude(double s1x, double s1y, double s2x, double s2y) {
	return (sqrt((s1x - s2x) * (s1x - s2x) + (s1y - s2y) * (s1y - s2y)));
}

//moment of inertia for solid sphere
double ballMoI(double r, double m)
{
	return(2 * m * r * r / 5);
}

//magnitude of cross product of two vectors in 2d
double crossProductMag(double x1, double y1, double x2, double y2)
{
	return(findMagnitude(y1 - y2, -(x1 - x2), x1 * y2 - y1 - x2));
}

bool spherecollided2d(double sx1, double sy1, double sx2, double sy2, double radius) {
	return (findMagnitude(sx1, sy1, sx2, sy2) < 2 * radius + epsilon); // note that magnitude is always positive
}

// the function for the force exerted by the spring, for one dimension
double F_s(double vx, double compression) {
	// for now its just a standard ideal spring, with force given by hooke's law
	double k = 100; // keep it 100
	if (vx == 0) { return 0; }
	else { return (-1) * vx / fabs(vx) * k * compression; }
}

double findTorque(double Fx, double Fy, double rx, double ry)
{
	double sinT = crossProductMag(Fx, Fy, rx, ry) / (findMagnitude(Fx, Fy) * findMagnitude(rx, ry));
	return findMagnitude(rx, ry) * findMagnitude(Fx, Fy) * sinT;
}

// with a spring, using just basic kinematics
void springcollision() {
	double compression = 0;

	double m1 = 1, m2 = 1;
	double s1x = -3, s1y = 4;
	double s2x = -3, s2y = -4;
	double v1x = 4, v1y = -6;
	double v2x = 4, v2y = 6;
	double radius = 1;
	double tinc = 0.001;

	double t = 0, printinc = 9;
	double tcollision = 3000;
	bool collision = false;
	do {
		t += tinc;
		printinc++;

		// check if collided
		if (spherecollided2d(s1x, s1y, s2x, s2y, radius)) {
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
			cout << "time: " << t << "seconds\n";
			cout << "s1: " << s1x << ", " << s1y << "\ns2: " << s2x << ", " << s2y << '\n';
			cout << "v1: " << v1x << ", " << v1y << "\nv2: " << v2x << ", " << v2y << '\n';
			cout << "distance: " << findMagnitude(s1x, s1y, s2x, s2y) << '\n';

			printinc = 0;
		}

	} while (t < tcollision + 1);
}

// glancing collision, end rotation, no initial rotation
void glancingcollision() {
	double compression = 0;

	double m1 = 1, m2 = 1;
	double s1x = -3, s1y = 4;
	double s2x = -3, s2y = -4;
	double v1x = 4, v1y = -6;
	double v2x = 4, v2y = 6;
	double radius = 1;
	double i1 = ballMoI(m1, radius);
	double i2 = ballMoI(m2, radius);
	double omega1 = 0, omega2 = 0;
	double tinc = 0.001;

	double t = 0, printinc = 9;
	double tcollision = 3000;
	bool collision = false;
	do {
		t += tinc;
		printinc++;

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
		}

		s1x += v1x * tinc;
		s1y += v1y * tinc;
		s2x += v2x * tinc;
		s2y += v2y * tinc;

		if (printinc == 10) {
			cout << "time: " << t << "seconds\n";
			cout << "s1: " << s1x << ", " << s1y << "\ns2: " << s2x << ", " << s2y << '\n';
			cout << "v1: " << v1x << ", " << v1y << "\nv2: " << v2x << ", " << v2y << '\n';
			cout << "distance: " << findMagnitude(s1x, s1y, s2x, s2y) << '\n';

			printinc = 0;
		}

	} while (t < tcollision + 1);
}

int main() {
	ofstream table; // for csv record of data

	//onedimension();
	//twodimension();
	springcollision();


	return 0;
}