#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include "qiaoliang/qiaoliang_define.h"

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const int	REQ_SONAR_INIT_COUNT = 10;
static const uint32_t	SONAR_TIMEOUT = 5000000;	// 2.0 s
static const float	SONAR_MAX_INIT_STD = 0.3f;	// meters

void BlockLocalPositionEstimator::sonarInit()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (_sonarStats.getCount() == 0) {
		_time_init_sonar = _timeStamp;
	}

	if (sonarMeasure(y) != OK) {
		return;
	}

	// if finished
	if (_sonarStats.getCount() > REQ_SONAR_INIT_COUNT) {
		if (_sonarStats.getStdDev()(0) > SONAR_MAX_INIT_STD) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar init std > min");
			_sonarStats.reset();

		} else if ((_timeStamp - _time_init_sonar) > SONAR_TIMEOUT) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar init timeout ");
			_sonarStats.reset();

		} else {
			PX4_INFO("[lpe] sonar init "
				 "mean %d cm std %d cm",
				 int(100 * _sonarStats.getMean()(0)),
				 int(100 * _sonarStats.getStdDev()(0)));
			_sensorTimeout &= ~SENSOR_SONAR;
			_sensorFault &= ~SENSOR_SONAR;
		}
	}
}

int BlockLocalPositionEstimator::sonarMeasure(Vector<float, n_y_sonar> &y)
{
// measure
#if __DAVID_DISTANCE__
//uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum
//uint8 ROTATION_DOWNWARD_FACING = 25 # MAV_SENSOR_ROTATION_PITCH_270
//uint8 ROTATION_UPWARD_FACING   = 24 # MAV_SENSOR_ROTATION_PITCH_90
//uint8 ROTATION_BACKWARD_FACING = 12 # MAV_SENSOR_ROTATION_PITCH_180
//uint8 ROTATION_FORWARD_FACING  = 0  # MAV_SENSOR_ROTATION_NONE
//uint8 ROTATION_LEFT_FACING     = 6  # MAV_SENSOR_ROTATION_YAW_270
//uint8 ROTATION_RIGHT_FACING    = 2  # MAV_SENSOR_ROTATION_YAW_90

	float d = - _sub_sonar->get().current_distance;
//	PX4_ZK("distance_sensor_s::ROTATION_UPWARD_FACING %.2f",(double)d);

#else/*__DAVID_DISTANCE__*/
	float d = _sub_sonar->get().current_distance;
#endif/*__DAVID_DISTANCE__*/
	//PX4_ZK("current_distance %.2f",(double)d);

	float eps = 0.01f;	// 1 cm
	float min_dist = _sub_sonar->get().min_distance + eps;
#if __DAVID_DISTANCE__
	float max_dist = _sonar_max_switch.get();

#else/* __DAVID_DISTANCE__*/
	float max_dist = _sub_sonar->get().max_distance - eps;
#endif/* __DAVID_DISTANCE__*/


	// prevent driver from setting min dist below eps
	if (min_dist < eps) {
		min_dist = eps;
	}

	// check for bad data
#if __DAVID_DISTANCE__
	if (fabsf(d) > max_dist || fabsf(d) < min_dist) 
#else
	if (d > max_dist || d < min_dist) 
#endif/**/
	{
		return -1;
	}

	// update stats
	_sonarStats.update(Scalarf(d));
	_time_last_sonar = _timeStamp;
//PX4_ZK("-aa-_time_last_sonar %lld",_time_last_sonar);
	y.setZero();
	y(0) = (d + _sonar_z_offset.get()) *
	       cosf(_eul(0)) *
	       cosf(_eul(1));
	
	return OK;
}

void BlockLocalPositionEstimator::sonarCorrect()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (sonarMeasure(y) != OK) { return; }

	// do not use sonar if lidar is active and not faulty or timed out
	if (_lidarUpdated
	    && !(_sensorFault & SENSOR_LIDAR)
	    && !(_sensorTimeout & SENSOR_LIDAR)) { return; }

	// calculate covariance
	float cov = _sub_sonar->get().covariance;

	if (cov < 1.0e-3f) {
		// use sensor value if reasoanble
		cov = _sonar_z_stddev.get() * _sonar_z_stddev.get();
	}

	// sonar measurement matrix and noise matrix
	Matrix<float, n_y_sonar, n_x> C;
	C.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	C(Y_sonar_z, X_z) = -1;	// measured altitude, negative down dir.
	C(Y_sonar_z, X_tz) = 1;	// measured altitude, negative down dir.

	// covariance matrix
	SquareMatrix<float, n_y_sonar> R;
	R.setZero();
	R(0, 0) = cov;

	// residual
	Vector<float, n_y_sonar> r = y - C * _x;
	// residual covariance
	Matrix<float, n_y_sonar, n_y_sonar> S = C * _P * C.transpose() + R;

	// publish innovations
	_pub_innov.get().hagl_innov = r(0);
	_pub_innov.get().hagl_innov_var = S(0, 0);

	// residual covariance, (inverse)
	Matrix<float, n_y_sonar, n_y_sonar> S_I = inv<float, n_y_sonar>(S);

	// fault detection
	float beta = (r.transpose()  * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_sonar]) {
		if (!(_sensorFault & SENSOR_SONAR)) {
			_sensorFault |= SENSOR_SONAR;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_SONAR) {
		_sensorFault &= ~SENSOR_SONAR;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar OK");
	}

	// kalman filter correction if no fault
	if (!(_sensorFault & SENSOR_SONAR)) {
		Matrix<float, n_x, n_y_sonar> K =
			_P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::sonarCheckTimeout()
{
//PX4_ZK("_timeStamp %lld _time_last_sonar %lld",_timeStamp,_time_last_sonar);
	if (_timeStamp - _time_last_sonar > SONAR_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_SONAR)) {
			_sensorTimeout |= SENSOR_SONAR;
			_sonarStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar timeout ");
		}
	}
}
