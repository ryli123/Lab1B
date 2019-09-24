/*
 * Alternate version of the collision -
 * not sure if it takes spring
 */

#include "header.h"
#ifndef RUN
#include "stdafx.h"
#include <iostream>
#include "collisionCheck.h"
#include "findAngle.h"
#include <math.h>
#include <fstream>

using namespace std;

bool collisionCheck(float s0x1, float s0y1, float s0x2, float s0y2, float ballRad)
{
	if (sqrt((s0x2 - s0x1) * (s0x2 - s0x1) + (s0y2 - s0y1) * (s0y2 - s0y1)) <= ballRad * 2)
		return true;
	else
		return false;
}

float findAngle(float sx1, float sy1, float sx2, float sy2) { //this program takes 2 sets of coordinates
	float y = sy2 - sy1; //delta y
	float x = sx2 - sx1; //delta x
	float angle = 0;
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

int main()
{
	float k; //For reference, k, mass, and ballRad can all be inputted
	float mass1 = 0.018, mass2 = 0.018; //A default value
	float ballRad = 0.0315; //A default value
	float sx1, sy1, sx2, sy2, scx1 = 0, scy1 = 0, scx2 = 0, scy2 = 0;
	float v1, v2, theta1, theta2, vx1, vy1, vx2, vy2;
	float ax1 = 0, ax2 = 0, ay1 = 0, ay2 = 0;
	float angx1 = 0, angx2 = 0, angv1, angv2;
	float t = 0, tinc = 0.001; //time increases by 0.001 sec every time the program loops
	float collisionTime = 4.0; //set an initial collisionTime to 4 (collisionTime is used in the conditions to end the while loop). This ensures that if the balls never collide, the program automatically ends after 6s.
	float pi = atan(1) * 4; //there's no pi constant 
	float ballAngle1 = 0, ballAngle2 = 0;
	float ballDist = 0, ballDistx = 0, ballDisty = 0; //initialize these distance variables at 0 to avoid errors with the program
	int increment = 9; //start an increment at 9, will use in the while loop
	ofstream table;

	// inputs
	cout << "Please enter a mass for ball 1 (kg): ";
	cin >> mass1;
	cout << "Please enter a mass for ball 2 (kg): ";
	cin >> mass2;
	cout << "Please enter radius of ball (m): ";
	cin >> ballRad;
	cout << "Please enter initial x coordinates for object 1 (m): "; //inputs for initial displacement of two bodies
	cin >> sx1;
	cout << "Please enter initial y coordinates for object 1 (m): ";
	cin >> sy1;
	cout << "Please enter initial x coordinates for object 2 (m): ";
	cin >> sx2;
	cout << "Please enter initial y coordinates for object 2 (m): ";
	cin >> sy2;

	cout << "Please enter initial velocity magnitude for object 1 (m/s): "; //initial velocity for 1st body
	cin >> v1;
	cout << "Please enter initial velocity angle for object 1 (degrees): ";
	cin >> theta1;
	vx1 = v1 * cos(theta1 * pi / 180.0);
	vy1 = v1 * sin(theta1 * pi / 180.0);

	cout << "Please enter initial velocity magnitude for object 2 (m/s) (Enter 0 for a still object): "; //initial velocity for second body (can choose between moving and stationary object)
	cin >> v2;
	if (v2 != 0.0) {
		cout << "Please enter initial velocity angle for object 2 (degrees): ";
		cin >> theta2;
		vx2 = v2 * cos(theta2 * pi / 180.0);
		vy2 = v2 * sin(theta2 * pi / 180.0);
	}
	else {
		vx2 = 0.0;
		vy2 = 0.0;
	}

	cout << "Please enter initial rotational velocity for object 1 (rad/s): ";
	cin >> angv1;
	cout << "Please enter initial rotational velocity for object 2 (rad/s): ";
	cin >> angv2;

	table.open("table.csv"); //create a csv file, allows for easier formatting
	table << "t,sx1,sy1,sx2,sy2,vx1,vy1,vx2,vy2,angx1,angx2,angv1,angv2\n"; //table axis labels
	// while loop
	do {
		t += tinc; //increase time interval
		//increment++; //increase increment (0 to 10) (gets to 10 on first loop)
		if (collisionCheck(sx1, sy1, sx2, sy2, ballRad)) { //if the two balls are found to be in contact
			ballAngle1 = findAngle(sx1, sy1, sx2, sy2); //find the angle between the two bodies
			if (ballAngle1 > pi)
				ballAngle2 = ballAngle1 - pi;
			else
				ballAngle2 = ballAngle1 + pi; //reverse the angles between the two bodies 
			ballDistx = sx2 + ballRad * cos(ballAngle2) - (sx1 + ballRad * cos(ballAngle1)); //takes the "overlap distance" between the two balls' radii in x and y
			ballDisty = sy2 + ballRad * sin(ballAngle2) - (sy1 + ballRad * sin(ballAngle1));
			scx1 = sx1 + ballRad * cos(ballAngle1);
			scy1 = sy1 + ballRad * sin(ballAngle1);
			scx2 = sx2 + ballRad * cos(ballAngle2);
			scy2 = sy2 + ballRad * sin(ballAngle2);
			ballDist = sqrt((ballDistx) * (ballDistx)+(ballDisty) * (ballDisty)); //magnitude of the two taken components
			k = 1.67e7 * ballDist * ballDist - 3e5 * ballDist + 1.9e3;
			ax1 = k * (ballDistx) / mass1; //find accelerations based on the equation ma = -kx (rearranging to isolate a)
			ay1 = k * (ballDisty) / mass1;
			ax2 = k * (-ballDistx) / mass2;
			ay2 = k * (-ballDisty) / mass2;
			vx1 += ax1 * tinc; //find velocities based on these accelerations given
			vy1 += ay1 * tinc;
			vx2 += ax2 * tinc;
			vy2 += ay2 * tinc;
			angv1 += k * ballDist * tinc * cos(findAngle(sy1, sx1, sy2, sx2)) / ((ballRad - ballDist) * mass2); //find angular velocity using the relation w = v/r
			angv2 += k * ballDist * tinc * cos(fmod(findAngle(sy1, sx1, sy2, sx2) + pi, 2 * pi)) / ((ballRad - ballDist) * mass1); //v can be represented as (F/m)cos(theta)*t (this is the tangential velocity at any given time)
			collisionTime = t; //set collision time for the while loop
		}
		sx1 += vx1 * tinc; //find displacement based on current velocity
		sy1 += vy1 * tinc;
		sx2 += vx2 * tinc;
		sy2 += vy2 * tinc;
		angx1 = fmod(angx1 + angv1 * tinc, 2 * pi); //find angular displacement (radians) based on current angular velocity
		angx2 = fmod(angx2 + angv2 * tinc, 2 * pi);
		if (increment == 10) //only prints every 10 ticks, doing this because the time increment is small (0.001s) to ensure accuracy. This prevents a lot of unnecessary data being outputted, while still modelling the compression of the two balls.
		{
			table << round(t * 1000) / 1000 << ",";
			table << round(sx1 * 1000) / 1000 << "," << round(sy1 * 1000) / 1000 << "," << round(sx2 * 1000) / 1000 << "," << round(sy2 * 1000) / 1000 << ","; //print out values for displacement of two balls
			table << round(vx1 * 1000) / 1000 << "," << round(vy1 * 1000) / 1000 << "," << round(vx2 * 1000) / 1000 << "," << round(vy2 * 1000) / 1000 << ","; //print out values for velocity (components) of two balls
			table << round(angx1 * 1000) / 1000 << "," << round(angx2 * 1000) / 1000 << "," << round(angv1 * 1000) / 1000 << "," << round(angv2 * 1000) / 1000 << ","; //print out angular values
			if (collisionCheck(sx1, sy1, sx2, sy2, ballRad))
				table << round(scx1 * 1000) / 1000 << "," << round(scy1 * 1000) / 1000 << "," << round(scx2 * 1000) / 1000 << "," << round(scy2 * 1000) / 1000 << ","; //print out compression values if balls are in contact
			table << "\n";
			increment = 0; //reset increment to 0
		}
	} while (t < collisionTime + 2.0); //automatically stops giving data 2 sec after collision or after 6 sec if balls don't collide
	table.close();
	return 0;
}
#endif