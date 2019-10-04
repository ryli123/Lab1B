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

 //#include "stdafx.h"
#include "Windows.h"
//#include "collisionCheck.h"
//#include "findAngle.h"
#include <math.h>

using namespace std;
constexpr double pi = M_PI;	// renaming M_PI constant to pi (3.14...)
const double epsilon = 0.001; // margin of error

// helper method, replaces prompt for values
double prompt(string s) {
	cout << "Enter " << s;
	double x{};
	cin >> x;

	return x;
}

// helper method to evaluate collision
bool almostSame(float a, float b) {
	return fabs(a - b) <= ((fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

double findAngle(double sx1, double sy1, double sx2, double sy2) { //this program takes 2 sets of coordinates
	double y = sy2 - sy1; //delta y
	double x = sx2 - sx1; //delta x
	double angle = 0;
	if (x == 0) { //if x = 0, atan doesn't work normally as denominator = 0. Hence, we must use special cases.
		if (y > 0)
			angle = atan(1) * 2;
		else if (y < 0)
			angle = atan(1) * 6;
	}
	else { //considers all cases so as to avoid confusion with the atan function (e.g. if both x and y are negative)
		if (x < 0 && y >= 0 || x < 0 && y < 0)
			angle = atan(y / x) + atan(1) * 4;
		else if (x > 0 && y < 0)
			angle = atan(y / x) + atan(1) * 8;
		else if (x > 0 && y > 0)
			angle = atan(y / x);
	}
	return angle;
}

double findMagnitude(double s1x, double s1y, double s2x, double s2y) {
	return (sqrt((s1x - s2x) * (s1x - s2x) + (s1y - s2y) * (s1y - s2y)));
}


// checks if positions are same --> collided
bool collided(double sx1, double sx2) {
	return almostSame(sx1, sx2);
}

bool spherecollided(double sx1, double sx2, double radius) {
	return ((sx1 - sx2) <= 2 * radius + epsilon) && ((sx1 - sx2) >= (-1) * 2 * radius - epsilon);
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

// testing in 1d
void onedimension() {
	double mass1 = 1, mass2 = 1;
	double rad = 1;
	double sx1 = 12, sx2 = 321;
	double v1 = 17, v2 = -16;
	double t = 0, tinc = 0.001, printinc = 0, tcollision = 3000; // default increment .001 s
	bool collision = false;

	// simulation with increment increm
	do {
		t += tinc;
		printinc++;

		// check collided
		if (collided(sx1, sx2) && !collision) {
			collision = true;
			cout << "\nCollided.\n";

			tcollision = t;
			v1 = (mass1 - mass2) * v1 / (mass1 + mass2) + 2 * mass1 * v2 / (mass1 + mass2);
			v2 = (mass1 - mass2) * v2 / (mass1 + mass2) + 2 * mass1 * v1 / (mass1 + mass2);
		}

		sx1 += v1 * tinc;
		sx2 += v2 * tinc;

		if (printinc == 10) {
			cout << "s1: " << sx1 << "\ns2: " << sx2 << '\n';

			printinc = 0;
		}

	} while (t < tcollision + 1);

	// have to use compression to find bounce back
	// last night took an L but tonight I bounce back
}

// COPIED FROM OTHER.CPP 



void twodimension() {
	double m1 = 1, m2 = 1;
	double s1x = -3, s1y = -4;
	double s2x = 4, s2y = 3;
	double v1x = 3, v1y = 4;
	double v2x = -4, v2y = -3;
	double radius = 1;
	double tinc = 0.001;

	double t = 0, printinc = 0;
	double tcollision = 3000;
	bool collision = false;
	do {
		t += tinc;
		printinc++;

		// check if collided
		if (spherecollided2d(s1x, s1y, s2x, s2y, radius) && !collision) {
			collision = true;
			cout << "\nCollided.\n";

			tcollision = t;
			v1x = v1x - (2 * m2) / (m1 + m2) * (((v1x - v2x) * (s1x - s2x) + (v1y - v2y) * (s1y - s2y)) / (pow(findMagnitude(s2x, s2y, s1x, s1y), 2))) * (s1x - s2x);
			v2x = v2x - (2 * m1) / (m1 + m2) * (((v1x - v2x) * (s1x - s2x) + (v1y - v2y) * (s1y - s2y)) / (pow(findMagnitude(s2x, s2y, s1x, s1y), 2))) * (s2x - s1x);

			v1y = v1y - (2 * m2) / (m1 + m2) * (((v1x - v2x) * (s1x - s2x) + (v1y - v2y) * (s1y - s2y)) / (pow(findMagnitude(s2x, s2y, s1x, s1y), 2))) * (s1y - s2y);
			v2y = v2y - (2 * m1) / (m1 + m2) * (((v1x - v2x) * (s1x - s2x) + (v1y - v2y) * (s1y - s2y)) / (pow(findMagnitude(s2x, s2y, s1x, s1y), 2))) * (s2y - s1y);
		}

		s1x += v1x * tinc;
		s1y += v1y * tinc;
		s2x += v2x * tinc;
		s2y += v2y * tinc;

		if (printinc == 10) {
			cout << "s1: " << s1x << ", " << s1y << "\ns2: " << s2x << ", " << s2y << '\n';
			cout << "total velocity: " << (v1x + v1y + v2x + v2y) << '\n'; // tells you if there's smth wrong

			printinc = 0;
		}
	} while (t < tcollision + 1);
}

// with a spring, using just basic kinematics
void springcollision() {
	double k = 10;
	double compression = 0;

	double m1 = 1, m2 = 1;
	double s1x = -3, s1y = -4;
	double s2x = 6, s2y = 3;
	double v1x = 3, v1y = 4;
	double v2x = -4, v2y = -3;
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

			s1x += 0.5 * F_s(v1x, compression) * tinc * tinc;
			s1y += 0.5 * F_s(v1y, compression) * tinc * tinc;
			s2x += 0.5 * F_s(v2x, compression) * tinc * tinc;
			s2y += 0.5 * F_s(v2y, compression) * tinc * tinc;

			// assuming that both balls are compressed the same amount
			compression = (2 * radius - findMagnitude(s1x, s1y, s2x, s2y)) / 2;
			v1x = v1x + F_s(v1x, compression) / m1;
			v1y = v1y + F_s(v1y, compression) / m1;
			v2x = v2x + F_s(v2x, compression) / m2;
			v2y = v2y + F_s(v2y, compression) / m2;
		}


		if (printinc == 10) {
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