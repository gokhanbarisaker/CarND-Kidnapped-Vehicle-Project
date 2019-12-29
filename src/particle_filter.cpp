/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[], int num_particles, bool debug)
{
  /**
   * DONE: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * DONE: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  if (debug)
  {
    std::cout << "Initializing " << num_particles << " particles...." << std::endl;
  }

  this->debug = debug;
  this->num_particles = num_particles;
  weights.resize(num_particles);
  particles.resize(num_particles);

  double init_weight = 1.0;

  std::default_random_engine random_generator;

  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i)
  {
    double sample_x = dist_x(random_generator);
    double sample_y = dist_y(random_generator);
    double sample_theta = dist_theta(random_generator);

    Particle p = {
        i,
        sample_x,
        sample_y,
        sample_theta,
        init_weight,
        std::vector<int>(),
        std::vector<double>(),
        std::vector<double>()};

    particles[i] = p;
    weights[i] = init_weight;
  }

  if (debug)
  {
    std::cout << "Initialized << " << particles.size() << " particles!" << std::endl;
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  /**
   * DONE: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  // TODO: Check theta = 0 case

  if (debug)
  {
    std::cout << "Predicting...." << std::endl;
  }

  // TODO: Yaw rate is going off the charts!!!

  std::normal_distribution<double> noise_dist_x(0.0, std_pos[0]);
  std::normal_distribution<double> noise_dist_y(0.0, std_pos[1]);
  std::normal_distribution<double> noise_dist_theta(0.0, std_pos[2]);

  std::default_random_engine random_generator;
  size_t num_particles = particles.size();

  for (size_t i = 0; i < num_particles; ++i)
  {
    Particle p = particles[i];

    double x_predicted;
    double y_predicted;
    double theta_predicted;

    if (std::fabs(yaw_rate) < 0.0001)
    {
      double distance = velocity * delta_t;

      theta_predicted = p.theta;

      x_predicted = p.x + (distance * std::cos(theta_predicted));
      y_predicted = p.y + (distance * std::sin(theta_predicted));
    }
    else
    {
      theta_predicted = p.theta + (yaw_rate * delta_t);

      x_predicted = p.x + ((velocity / yaw_rate) * (std::sin(theta_predicted) - std::sin(p.theta)));
      y_predicted = p.y + ((velocity / yaw_rate) * (std::cos(p.theta) - std::cos(theta_predicted)));
    }

    p.x = x_predicted + noise_dist_x(random_generator);
    p.y = y_predicted + noise_dist_y(random_generator);
    p.theta = theta_predicted + noise_dist_theta(random_generator);
    particles[i] = p;
  }

  if (debug)
  {
    std::cout << "Predicted!" << std::endl;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> inrange_landmark_observations,
                                     vector<LandmarkObs> &map_observations)
{
  /**
   * DONE: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  for (size_t i = 0; i < map_observations.size(); ++i)
  {
    LandmarkObs &observation = map_observations[i];

    double dist_min = std::numeric_limits<double>::max();
    observation.id = -1;

    for (size_t j = 0; j < inrange_landmark_observations.size(); ++j)
    {
      LandmarkObs &inrange_landmarks_observation = inrange_landmark_observations[j];

      double dist_cur = dist(
          inrange_landmarks_observation.x,
          inrange_landmarks_observation.y,
          observation.x,
          observation.y);

      if (dist_cur <= dist_min)
      {
        dist_min = dist_cur;
        observation.id = inrange_landmarks_observation.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  /**
   * DONE: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  if (debug)
  {
    std::cout << "Updating weights...." << std::endl;
  }

  for (size_t i = 0; i < particles.size(); ++i)
  {
    Particle &p = particles[i];

    // List of nearby/in-range observations from sensors data. Items are in map coordinates.
    std::vector<LandmarkObs> map_observations(observations.size());

    for (size_t j = 0; j < observations.size(); ++j)
    {
      LandmarkObs const &observation = observations[j];
      LandmarkObs &map_observation = map_observations[j];

      double x = p.x + (std::cos(p.theta) * observation.x) - (std::sin(p.theta) * observation.y);
      double y = p.y + (std::sin(p.theta) * observation.x) + (std::cos(p.theta) * observation.y);

      map_observation.id = observation.id;
      map_observation.x = x;
      map_observation.y = y;
    }

    if (debug)
    {
      std::cout << "Transformed " << map_observations.size() << " observations" << std::endl;
    }

    std::vector<Map::single_landmark_s> const &landmarks = map_landmarks.landmark_list;
    std::vector<LandmarkObs> inrange_landmark_observations;

    for (size_t j = 0; j < landmarks.size(); ++j)
    {
      Map::single_landmark_s const &landmark = landmarks[j];

      if (dist(p.x, p.y, landmark.x_f, landmark.y_f) < sensor_range)
      {
        inrange_landmark_observations.push_back({landmark.id_i,
                                                 landmark.x_f,
                                                 landmark.y_f});
      }
    }

    if (debug)
    {
      std::cout << "Landmarks in range: " << inrange_landmark_observations.size() << std::endl;
    }

    dataAssociation(inrange_landmark_observations, map_observations);

    if (debug)
    {
      std::cout << "Association complete!!!!" << std::endl;
    }

    double weight = 1.0;

    for (size_t j = 0; j < map_observations.size(); ++j)
    {
      LandmarkObs &map_observation = map_observations[j];
      Map::single_landmark_s nearest_landmark = map_landmarks.landmark_list[map_observation.id - 1];

      weight *= gaussian2d(
          nearest_landmark.x_f,
          nearest_landmark.y_f,
          std_landmark[0],
          std_landmark[1],
          map_observation.x,
          map_observation.y);
    }

    p.weight = weight;
    weights[i] = weight;
  }

  if (debug)
  {
    std::cout << "Updated weights!" << std::endl;
  }
}

void ParticleFilter::resample()
{
  /**
   * DONE: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  if (debug)
  {
    std::cout << "Resampling ..." << std::endl;
  }

  std::default_random_engine random_generator;
  std::discrete_distribution<size_t> weight_distribution(weights.begin(), weights.end());
  std::vector<Particle> resampled_particles(particles.size());

  for (size_t i = 0; i < particles.size(); ++i)
  {
    resampled_particles[i] = particles[weight_distribution(random_generator)];
    resampled_particles[i].id = i;
  }

  particles = resampled_particles;

  if (debug)
  {
    std::cout << "Resampled ..." << std::endl;
  }
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}