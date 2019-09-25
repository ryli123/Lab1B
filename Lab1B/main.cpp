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

int main() {
	// mass, radius, positions + collision positions
	double mass1 = 1, mass2 = 1;	// default value
	double rad = 3.14;				// default values
	double sx1, sy1, sx2, sy2, scx1, scy1, scx2, scy2;


	return 0;
}