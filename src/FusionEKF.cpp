#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;
	previous_timestamp_ = 0;

	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	R_laser_ << 0.0225, 0,
			0, 0.0225;
	R_radar_ << 0.09, 0, 0,
			0, 0.0009, 0,
			0, 0, 0.09;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	if (!is_initialized_) {
		ekf_.x_ = VectorXd(4);
		ekf_.P_ = MatrixXd(4, 4);
		ekf_.P_ << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			float ro = measurement_pack.raw_measurements_[0];
			float theta = measurement_pack.raw_measurements_[1];
			float ro_dot = measurement_pack.raw_measurements_[2];
			ekf_.x_ <<ro * cos(theta),ro * sin(theta), ro_dot * cos(theta), ro_dot * sin(theta);
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			float px = measurement_pack.raw_measurements_[0];
			float py = measurement_pack.raw_measurements_[1];
			ekf_.x_ <<px,py,0, 0;
		}

		// done initializing, no need to predict or update
		is_initialized_ = true;
		previous_timestamp_ = measurement_pack.timestamp_;
		return;
	}


	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, dt, 0,
			0, 1, 0, dt,
			0, 0, 1, 0,
			0, 0, 0, 1;
	float noise_ax = 9.0;
	float noise_ay = 9.0;
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
	ekf_.Predict();



	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		ekf_.H_ = MatrixXd(3,4);
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.R_ = R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	} else {
		ekf_.H_ = MatrixXd(2, 4);
		ekf_.H_ << 1, 0, 0, 0,
				0, 1, 0, 0;
		ekf_.R_ = R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
