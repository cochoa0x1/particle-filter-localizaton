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

void ParticleFilter::print_particles(){
	std::cout <<"----------particles------------" << std::endl;

	for(auto & p : particles){
		std::cout << "id: " << p.id << " x: " << p.x << " y: " << p.y << " theta: "
			<< p.theta << " w: " << p.weight << std::endl;
	}
}
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	//  Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	std::cout << "Initialize!" << std::endl;

	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_psi(theta, std[2]);

	num_particles = 10;

	for(int i =0; i< num_particles; i++){
		Particle p ={i,dist_x(gen),dist_y(gen),bound_angle(dist_psi(gen)),1.0};
		particles.push_back(p);
	}

	is_initialized = true;

	print_particles();

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	std::cout << "PREDICT\n";

	//std::cout << "before transform::::\n";
	//print_particles();
	//for each particle, predict next position and direction

	std::default_random_engine gen;

	//update the particles position
	for (auto & p: particles){
		//if zero yaw_rate
		if(std::abs(yaw_rate) < .00001f){
			p.x+=velocity*delta_t*cos(p.theta);
			p.y+=velocity*delta_t*sin(p.theta);
			//if the yaw isnt chaning then the yaw doesnt change: theta remains the same
		}else{
			p.x+=(velocity/yaw_rate)*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta));
			p.y+=(velocity/yaw_rate)*(cos(p.theta)-cos(p.theta+yaw_rate*delta_t));
			p.theta+=yaw_rate*delta_t;
		}

		//add noise
		std::normal_distribution<double> dist_x(p.x, std_pos[0]);
		std::normal_distribution<double> dist_y(p.y, std_pos[1]);
		std::normal_distribution<double> dist_psi(p.theta, std_pos[2]);

		p.x= dist_x(gen);
		p.y= dist_y(gen);
		p.theta= bound_angle(dist_psi(gen));
	}

	//std::cout << "after transform::::\n";
	//print_particles();
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	//assign predicted measurements to landmarks observation

	for(auto & pred : predicted){

			//get the nearest
			LandmarkObs nearest;
			double nearest_dist = std::numeric_limits<double>::max();

			for( auto & obs : observations){
				double dx = pred.x - obs.x;
				double dy = pred.y - obs.y;
				double d = sqrt(dx*dx+dy*dy);
				if ( d < nearest_dist){
					nearest_dist = d;
					nearest = obs;
				}

				//assign it
				pred = nearest;
			}
	}

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


	//Transform each observation from vehicle to map coordinates
	//Particle p = particles[0];

	for(auto & p : particles){

		long double total_log_weight =0.0;

		for(auto & obs: observations){
			LandmarkObs tobs = {obs.id, obs.x, obs.y};

			//transform observations (car coordinates) as if they were taken in particle coordinates
			//to map coordinates so we can match them to landmarks which are given in map coordinates
			tobs.x = p.x+obs.x*cos(p.theta) -obs.y*sin(p.theta);
			tobs.y = p.y+obs.x*sin(p.theta) +obs.y*cos(p.theta);

			//find the landmark that matches up to this
			Map::single_landmark_s nearest;
			double nearest_dist = std::numeric_limits<double>::max();

			for( auto & landmark : map_landmarks.landmark_list){
				double dx = tobs.x - landmark.x_f;
				double dy = tobs.y - landmark.y_f;
				double d = sqrt(dx*dx+dy*dy);

				if ( d < nearest_dist){
					nearest_dist = d;
					nearest = landmark;
				}
			}

			//std::cout << "nearest: " << nearest.id_i << std::endl;
			//calculate the log of the weight to avoid numerical issues

			double ndx = tobs.x - nearest.x_f;
			double ndy = tobs.y - nearest.y_f;

			//std::cout << "ndx: " << ndx << " ndy: " << ndy << std::endl;

			long double log_w = -1.0*log(2*M_PI*std_landmark[0]*std_landmark[1])
					- ( (ndx*ndx)/(2*std_landmark[0]*std_landmark[0])
						+ (ndy*ndy)/(2*std_landmark[1]*std_landmark[1])
					);

			//std::cout << "logw " << log_w <<  " totalw: " << total_log_weight << std::endl;

			total_log_weight+=log_w;
		}

		p.weight = exp(total_log_weight);
		//std::cout << "----->weight: " << p.weight <<  std::endl;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//std::cout << "before resample: " << std::endl;
	//print_particles();

	//new vector of particles
	std::vector<Particle> new_particles;

	//make an array of weights
 	std::vector<double> ws;

	for(auto & p: particles){
		ws.push_back(p.weight);
	}

	std::default_random_engine gen;
	std::discrete_distribution<> sampler(ws.begin(), ws.end());

	for(int i=0; i< num_particles; i++){
		//sample from our old particles
		int idx = sampler(gen);
		Particle p = particles[idx];
		p.weight = 1.0; //reset the weight
		new_particles.push_back(p);

	}

	particles = new_particles;
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
