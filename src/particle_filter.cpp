/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // num of particules @TODO why 1k?
  num_particles = 1000;

  // Given std deviation and random
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  default_random_engine gen;

  // create gaussian distributions
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // init weights and particules
  for(int i = 0; i < num_particles; i++){
    weights.push_back(1.0);
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    particles.push_back(particle);
  }

  // set is_initialized to true
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // Given std deviation and random
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];
  default_random_engine gen;

  // The equations for updating x, y and the yaw angle when the **yaw rate is not equal to zero**:
  // xf=x0+velocity/yaw_rate(sin(theta+yaw_rate*dt)-sin(theta))
  // yf=y0+velocity/yaw_rate(cos(theta)-cos(theta+yaw_rate*dt))
  // thetaf=theta+yaw_rate*dt

  for(int i = 0; i < num_particles; i++){

    Particle p = particles[i];

    // **yaw rate is not equal to zero**:
    if (fabs(yaw_rate) > 0.001) {
      p.x = p.x + velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y = p.y + velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
      p.theta = p.theta + yaw_rate * delta_t;
    } else {
      p.x = p.x + velocity * delta_t * cos(p.theta);
      p.y = p.y + velocity * delta_t * sin(p.theta);
      p.theta = p.theta;
    }

    // adding noise, mean updated value, std_dev given by params
    normal_distribution<double> dist_x(p.x, std_x);
    normal_distribution<double> dist_y(p.y, std_y);
    normal_distribution<double> dist_theta(p.theta, std_theta);
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    std::vector<LandmarkObs> observations, Map map_landmarks) {
  // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
  //   for the fact that the map's y-axis actually points downwards.)
  //   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
  // You don't need to modify this file.
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::app);
  for (int i = 0; i < num_particles; ++i) {
    dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
  }
  dataFile.close();
}
