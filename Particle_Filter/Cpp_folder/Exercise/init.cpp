/**
 * print_samples.cpp
 *
 * Print out to the terminal 3 samples from a normal distribution with
 * mean equal to the GPS position and IMU heading measurements and
 * standard deviation of 2 m for the x and y position and 0.05 radians
 * for the heading of the car.
 *
 * Author: Tiffany Huang
 */

#include <iostream>
#include <random> // Need this for sampling from distributions

using std::normal_distribution;

/**
 * Prints samples of x, y and theta from a normal distribution
 * @param gps_x   GPS provided x position
 * @param gps_y   GPS provided y position
 * @param theta   GPS provided yaw
 */
void printSamples(double gps_x, double gps_y, double theta);


int main() {

	// Set GPS provided state of the car.
	double gps_x = 4983;
	double gps_y = 5029;
	double theta = 1.201;

	// Sample from the GPS provided position.
	printSamples(gps_x, gps_y, theta);

	return 0;
}


void printSamples(double gps_x, double gps_y, double theta) {
	std::default_random_engine gen;
	double std_x, std_y, std_theta;  // Standard deviations for x, y, and theta

	// TODO: Set standard deviations for x, y, and theta from the introduction
	std_x = 2.0;
	std_y = 2.0;
	std_theta = 0.05;

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(gps_x, std_x);

	// TODO: Create normal distributions for y and theta
	normal_distribution<double> dist_y(gps_y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);


	for (int i = 0; i < 3; ++i) {
		double sample_x, sample_y, sample_theta;

		// TODO: Sample from these normal distributions like this: 
		//   sample_x = dist_x(gen);
		//   where "gen" is the random engine initialized earlier.
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);


		// Print your samples to the terminal.
		std::cout << gen << std::endl;
		std::cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " "
			<< sample_theta << std::endl;
	}

	return;
}