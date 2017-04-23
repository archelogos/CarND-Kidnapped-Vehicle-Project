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
  num_particles = 500;

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
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.id = i;
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
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.

  vector<LandmarkObs> identified_p_observations;

  for (int i=0; i < predicted.size(); i++) {
    LandmarkObs prediction = predicted[i];
    // maximum number
    double closest_distance = numeric_limits<double>::max();
    int closest_observation_index = 0;

    for (int j = 0; j < observations.size(); j++){
      LandmarkObs observation = observations[j];
      if (observation.id == 0) {
        double distance = dist(observation.x, observation.y, prediction.x, prediction.y);
        if (distance < closest_distance) {
          // update then
          closest_distance = distance;
          closest_observation_index = j;
        }
      }
    }
    observations[closest_observation_index].id = prediction.id;
    identified_p_observations.push_back(observations[closest_observation_index]);
  }

  observations = identified_p_observations;

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

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

  // for each particle
  for(int i = 0; i < num_particles; i++){

    Particle p = particles[i];

    // for each observation, vehicle system to map system according to the given formula
    vector<LandmarkObs> p_observations;
    for (int j = 0; j < observations.size(); j++) {
      LandmarkObs observation;
      observation.x = observations[j].x * cos(p.theta) - observations[j].y * sin(p.theta) + p.x;
      observation.y = observations[j].x * sin(p.theta) + observations[j].y * cos(p.theta) + p.y;
      observation.id = 0; // to check the condition later in dataAssociation
      p_observations.push_back(observation);
    }

    // estimate the dataAssociation (nearest-neighbors) just in the sensor_range of this particle
    vector<LandmarkObs> predicted;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      // euclidean distance from helper functions
      if (dist(p.x, p.y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f) < sensor_range) {
        LandmarkObs landmark;
        landmark.id = map_landmarks.landmark_list[j].id_i;
        landmark.x = map_landmarks.landmark_list[j].x_f;
        landmark.y = map_landmarks.landmark_list[j].y_f;
        predicted.push_back(landmark);
      }
    }

    // now! find nearest neighbor (it updates p_observations)
    dataAssociation(predicted, p_observations);

    // calculate multi variate gaussian weight
    double weight_product = 1.0;
    for (int j; j < p_observations.size(); j++) {
      LandmarkObs measurement = p_observations[j];
      LandmarkObs predicted_measurement;

      // get predction for this measurement
      for (int k; k < predicted.size(); k++) {
        LandmarkObs prediction = predicted[k];
        if (measurement.id == prediction.id) {
          predicted_measurement = prediction;
        }
      }

      // bi-variate gaussian weight
      double mu_x = predicted_measurement.x;
      double mu_y = predicted_measurement.y;
      double x = measurement.x;
      double y = measurement.y;

      double c1 = 1.0 / (2.0 * M_PI * std_x * std_y);
      double c2 = pow(x - mu_x, 2) / pow(std_x, 2);
      double c3 = pow(y - mu_y, 2) / pow(std_y, 2);
      double weight = c1 * exp(-0.5 * (c2 + c3));

      if (weight < .0001)
        weight = .0001;

      weight_product *= weight;
    }

    // update particle weight
    p.weight = weight_product;

  } // end particles loop

}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // intialise resampling wheel
  double beta = 0.0;
  default_random_engine gen_a;
  uniform_int_distribution<int> dis(0, num_particles - 1);
  int index = dis(gen_a);

  vector<Particle> resampled_particles;
  resampled_particles.reserve(num_particles);

  // find max weight and create particle weights vectors
  double max_weight = 0.0;
  weights.clear();
  weights.reserve(num_particles);
  for (auto particle : particles) {
    if (particle.weight > max_weight)
      max_weight = particle.weight;
    weights.push_back(particle.weight);
  }

  // create random number generator for two times max weight
  default_random_engine gen_b;
  uniform_real_distribution<double> dis_real(0, 2.0 * max_weight);

  // resample
  for (auto particle : particles) {
    beta += dis_real(gen_b);
    while (weights[index] < beta) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }

  // update particles
  particles = resampled_particles;

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
