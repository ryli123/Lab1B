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

using namespace std;
constexpr double pi = M_PI;	// renaming M_PI constant to pi (3.14...)

// helper method, replaces prompt for values
double prompt(string s) {
	cout << "Enter " << s;
	double x{};
	cin >> x;

	return x;
}

// helper method to evaluate collision
bool almostSame(float a, float b, float epsilon) {
	return fabs(a - b) <= ((fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

// checks if positions are same --> collided
bool collided(double sx1, double sx2) {
	return almostSame(sx1, sx2, 1e-8);
}

// testing in 1d
void onedimension() {
	double mass1 = 1, mass2 = 1;
	double rad = 1;
	double sx1, sx2, scx1, scx2;

	double v1 = 1, v2 = 0;
	double t = 0, increm = 0.001; // default increment .001 s

	while (false) {

	}

	// have to use compression to find bounce back
	// last night took an L but tonight I bounce back
}

int main() {
	// mass, radius, positions + collision positions
	double mass1 = 1, mass2 = 1;	// default value
	double rad = 1;				// default values
	double sx1, sy1, sx2, sy2, scx1, scy1, scx2, scy2;
	ofstream table; // for csv record of data



	return 0;
}
// austin first edit