// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <iostream>
#include <fstream>
#include <vector>

// for file writing
vector<double> h, velocity_mag;

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    double h, kp, kh, error, delta, power_out;

    kp = 0.8;
    kh = 0.0169;
    delta = 0.481;
       

    h = position.abs() - MARS_RADIUS;
    error = -(0.5 + kh * h + velocity * position.norm());
    power_out = kp * error;

        if (power_out <= -delta) {
            throttle = 0;
        }
        else if (- delta < power_out < 1 - delta) {
            throttle = delta + power_out;
        }
        else if (power_out >= 1 - delta) {
            throttle = 1;
        }

}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // First we initialise and calculate each individual force 
    vector3d force_gravitational, force_thrust, force_drag;

  // Initialise constants and variables to be used in force calculations 
    double position_mag_sqrd, density;
    vector3d position_normal, net_force, acceleration;

    position_mag_sqrd = position.abs2();
    position_normal = position.norm();

 
    // for simplicity, assume projected area is 1 metre always
    force_gravitational = -(GRAVITY * MARS_MASS * (UNLOADED_LANDER_MASS + fuel * 100) / position_mag_sqrd) * position_normal;
    force_thrust = thrust_wrt_world();
    density = atmospheric_density(position);
    
    // Note: We find the projected area by using the formula for area of a circle, where the radius is 1 m
    if (parachute_status == DEPLOYED) {
        force_drag = -0.5 * density * (DRAG_COEF_LANDER + DRAG_COEF_CHUTE) * 3.14159265358979323846264 * velocity.abs2() * velocity.norm();
    }
    else {
        force_drag = -0.5 * density * DRAG_COEF_LANDER * 3.14159265358979323846264 * velocity.abs2() * velocity.norm();
    }
    

    net_force = force_gravitational + force_thrust + force_drag;

    /*// Eulers 
    acceleration = net_force / (UNLOADED_LANDER_MASS + fuel * 100);
    position = position + delta_t * velocity;
    velocity = velocity + delta_t * acceleration;*/

    // reset the h and velocity magnitude lists if simulation time is reset
    if (simulation_time == 0.0) {
        h.clear();
        velocity_mag.clear();
    }

    // writing the values of h and v.er (velocity magnitude) at each time step to a file
    double altitude;
    float i;

    altitude = position.abs() - MARS_RADIUS;
    h.push_back(altitude);
    velocity_mag.push_back(velocity * position.norm());


    // write the resulting data to a text file when the simulation has finished
    if (h.size() > 3674) {
        ofstream fout;
        fout.open("autopilot_scenario_1_take3.txt");
        if (fout) {
            for (i = 0; i < h.size(); i = i + 1) {
                fout << i / 10 << ' ' << h[i] / 1000 << ' ' << velocity_mag[i] << endl;
            }
        }
        else {
            cout << "Could not open trajectory file for writing" << endl;

        }
    }


    // Verlets
    vector3d position_previous, position_current;

    
    position_previous = position - delta_t * velocity;

    acceleration = net_force / (UNLOADED_LANDER_MASS + fuel * 100);
    position_current = position;
    position = (2 * position_current) - position_previous + (delta_t * delta_t) * acceleration;
    velocity = (1 / delta_t) * (position - position_current);





  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();


}



void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
