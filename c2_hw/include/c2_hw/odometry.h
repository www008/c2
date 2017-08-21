#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace diff_drive_motors {
namespace bacc = boost::accumulators;

class Odometry {
private:
	/// Rolling mean accumulator and window:
	typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
	typedef bacc::tag::rolling_window RollingWindow;

	/**
	 * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
	 * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
	 * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
	 */
	void integrateRungeKutta2(double linear, double angular) {
		const double direction = heading_ + angular * 0.5;

		/// Runge-Kutta 2nd order integration:
		x_       += linear * cos(direction);
		y_       += linear * sin(direction);
		heading_ += angular;
	}

	/**
	 * \brief Integrates the velocities (linear and angular) using exact method
	 * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
	 * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
	 */
	void integrateExact(double linear, double angular) {
		if (fabs(angular) < 1e-6)
		  integrateRungeKutta2(linear, angular);
		else
		{
		  /// Exact integration (should solve problems when angular is zero):
		  const double heading_old = heading_;
		  const double r = linear/angular;
		  heading_ += angular;
		  x_       +=  r * (sin(heading_) - sin(heading_old));
		  y_       += -r * (cos(heading_) - cos(heading_old));
		}
	}

	/**
	 *  \brief Reset linear and angular accumulators
	 */
	void resetAccumulators() {
		linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
		angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
	}

	/// Current timestamp:
	ros::Time timestamp_;

	/// Current pose:
	double x_;        //   [m]
	double y_;        //   [m]
	double heading_;  // [rad]

	/// Current velocity:
	double linear_;  //   [m/s]
	double angular_; // [rad/s]

	/// Wheel kinematic parameters [m]:
	double wheel_separation_;
	double wheel_radius_;

	/// Rolling mean accumulators for the linar and angular velocities:
	size_t velocity_rolling_window_size_;
	RollingMeanAcc linear_acc_;
	RollingMeanAcc angular_acc_;

	/// Integration funcion, used to integrate the odometry:
	boost::function<void(double, double)> integrate_fun_;
public:
	/**
	 * \brief Constructor
	 * Timestamp will get the current time value
	 * Value will be set to zero
	 * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
	 */
	Odometry(size_t velocity_rolling_window_size = 10) : timestamp_(0.0)
		, x_(0.0)
		, y_(0.0)
		, heading_(0.0)
		, linear_(0.0)
		, angular_(0.0)
		, wheel_separation_(0.0)
		, wheel_radius_(0.0)
		, velocity_rolling_window_size_(velocity_rolling_window_size)
		, linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
		, angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
		, integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2)) { }

	/**
	 * \brief Initialize the odometry
	 * \param time Current time
	 */
	void init(const ros::Time &time)  {
		// Reset accumulators and timestamp:
		resetAccumulators();
		timestamp_ = time;
	}

	/**
	 * \brief Updates the odometry class with latest wheels position
	 * \param left_pos  Left  wheel vel [m/s]
	 * \param meas_rwheel_vel Right wheel vel [m/s]
	 * \param time      Current time
	 * \return true if the odometry is actually updated
	 */
	bool update(double meas_lwheel_vel, double meas_rwheel_vel, const ros::Time &time) {
		/// Compute linear and angular diff:
		const double linear  = (meas_rwheel_vel + meas_lwheel_vel) * 0.5 ;
		const double angular = (meas_rwheel_vel - meas_lwheel_vel) / wheel_separation_;

		/// Integrate odometry:
		integrate_fun_(linear, angular);

		/// We cannot estimate the speed with very small time intervals:
		const double dt = (time - timestamp_).toSec();
		if (dt < 0.0001)
		  return false; // Interval too small to integrate with

		timestamp_ = time;

		/// Estimate speeds using a rolling mean to filter them out:
		linear_acc_(linear/dt);
		angular_acc_(angular/dt);

		linear_ = bacc::rolling_mean(linear_acc_);
		angular_ = bacc::rolling_mean(angular_acc_);

		return true;
	}

	/**
	 * \brief Updates the odometry class with latest velocity command
	 * \param linear  Linear velocity [m/s]
	 * \param angular Angular velocity [rad/s]
	 * \param time    Current time
	 */
	void updateOpenLoop(double linear, double angular, const ros::Time &time) {
	    /// Save last linear and angular velocity:
		linear_ = linear;
		angular_ = angular;

		/// Integrate odometry:
		const double dt = (time - timestamp_).toSec();
		timestamp_ = time;
		integrate_fun_(linear * dt, angular * dt);
	}

	/**
	 * \brief heading getter
	 * \return heading [rad]
	 */
	double getHeading() const {
		return heading_;
	}

	/**
	 * \brief x position getter
	 * \return x position [m]
	 */
	double getX() const {
		return x_;
	}

	/**
	 * \brief y position getter
	 * \return y position [m]
	 */
	double getY() const {
		return y_;
	}

	/**
	 * \brief linear velocity getter
	 * \return linear velocity [m/s]
	 */
	double getLinear() const {
		return linear_;
	}

	/**
	 * \brief angular velocity getter
	 * \return angular velocity [rad/s]
	 */
	double getAngular() const {
		return angular_;
	}

	/**
	 * \brief Sets the wheel parameters: radius and separation
	 * \param wheel_separation Seperation between left and right wheels [m]
	 * \param wheel_radius     Wheel radius [m]
	 */
	void setWheelParams(double wheel_separation, double wheel_radius) {
	    wheel_separation_ = wheel_separation;
		wheel_radius_     = wheel_radius;
	}

	/**
	 * \brief Velocity rolling window size setter
	 * \param velocity_rolling_window_size Velocity rolling window size
	 */
	void setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
	    velocity_rolling_window_size_ = velocity_rolling_window_size;
		resetAccumulators();
	}
};
}

#endif

