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
/*
void ParticleFilter::print_particles(){
	std::cout <<"----------particles------------" << std::endl;

	for(auto & p : particles){
		std::cout << "id: " << p.id << " x: " << p.x << " y: " << p.y << " theta: "
			<< p.theta << " w: " << p.weight << std::endl;
	}
}
*/
void ParticleFilter::init(double x, double y, double theta, double std[]) {

	//std::cout << "Initialize!" << std::endl;

	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_psi(theta, std[2]);

	num_particles = 20;

	for(int i =0; i< num_particles; i++){
		Particle p ={i,dist_x(gen),dist_y(gen),dist_psi(gen),1.0};
		particles.push_back(p);
	}

	is_initialized = true;

	//print_particles();

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

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
		p.theta= dist_psi(gen);
	}

	//std::cout << "after transform::::\n";
	//print_particles();
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	//assign predicted measurements to landmarks observation

	//NOTE, I ended up not using this
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

	for(auto & p : particles){

		long double total_log_weight =0.0; //use log weight to avoid numerical issues

		for(auto & obs: observations){
			LandmarkObs tobs = {obs.id, obs.x, obs.y};

			//transform observations (car coordinates) as if they were taken in particle coordinates
			//to map coordinates so we can match them to landmarks which are given in map coordinates
			tobs.x = p.x+obs.x*cos(p.theta) -obs.y*sin(p.theta);
			tobs.y = p.y+obs.x*sin(p.theta) +obs.y*cos(p.theta);

			//find the landmark that matches up to this by searching for the closest landmark
			//TODO make sure landmark is within sensor_range?
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
			//calculate the contribution to the total weight
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
	//std::cout << "before resample: " << std::endl;
	//print_particles();

	//new vector of particles
	std::vector<Particle> new_particles;
	//particle weights
 	std::vector<double> ws;

	for(auto & p: particles){
		ws.push_back(p.weight);
	}

	std::default_random_engine gen;
	std::discrete_distribution<> sampler(ws.begin(), ws.end());

	//sample num_particles new particles
	for(int i=0; i< num_particles; i++){
		//sample from our old particles
		int idx = sampler(gen);
		Particle p = particles[idx];
		p.weight = 1.0; //reset the weight
		new_particles.push_back(p);
	}

	//update our particles with new ones
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
