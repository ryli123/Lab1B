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

double findMagnitude(double s1x, double s1y, double s2x, double s2y) {
	return (sqrt((s1x - s2x) * (s1x - s2x) + (s1y - s2y) * (s1y - s2y)));
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

// with a spring, using just basic kinematics
void springcollision() {
	double compression = 0;

	double m1 = 1, m2 = 1;
	double s1x = -3, s1y = -4;
	double s2x = -3, s2y = 4;
	double v1x = 18, v1y = 24;
	double v2x = 18, v2y = -24;
	double radius = 1;
	double tinc = 0.001;

	double t = 0, printinc = 9;
	double tcollision = 3000;
	bool collision = false;
	do {
		t += tinc;
		printinc++;

		s1x += v1x * tinc;
		s1y += v1y * tinc;
		s2x += v2x * tinc;
		s2y += v2y * tinc;

		// check if collided
		if (spherecollided2d(s1x, s1y, s2x, s2y, radius)) {
			if (!collision) {
				cout << "\nCollided.\n";
				tcollision = t;
				collision = true;
			}

			s1x += 0.5 * F_s(v1x-v2x, compression) * tinc * tinc;
			s1y += 0.5 * F_s(v1y-v2y, compression) * tinc * tinc;
			s2x += 0.5 * F_s(v2x-v1x, compression) * tinc * tinc;
			s2y += 0.5 * F_s(v2y-v2x, compression) * tinc * tinc;

			// assuming that both balls are compressed the same amount
			compression = (2 * radius - findMagnitude(s1x, s1y, s2x, s2y)) / 2;
			v1x = v1x + tinc * F_s(v1x-v2x, compression) / m1;
			v1y = v1y - tinc * F_s(v1y-v2y, compression) / m1;
			v2x = v2x + tinc * F_s(v1x-v2x, compression) / m2;
			v2y = v2y - tinc * F_s(v1y-v2y, compression) / m2;

		}


		if (printinc == 10) {
			cout << "time: " << t << "seconds\n";
			cout << "s1: " << s1x << ", " << s1y << "\ns2: " << s2x << ", " << s2y << '\n';
			cout << "total velocity: " << (v1x + v1y + v2x + v2y) << '\n';
			cout << "distance: " << findMagnitude(s1x, s1y, s2x, s2y) << '\n';

			printinc = 0;
		}

	} while (t < tcollision + 1);
}

int main() {
	// mass, radius, positions + collision positions
	double mass1 = 1, mass2 = 1;	// default value
	double rad = 1;				// default values
	double sx1, sy1, sx2, sy2, scx1, scy1, scx2, scy2;
	ofstream table; // for csv record of data

	//onedimension();
	//twodimension();
	springcollision();


	return 0;
}