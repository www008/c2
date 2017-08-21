#ifndef C2_PID_H
#define C2_PID_H

#include <string>
#include <ros/ros.h>
#include <control_msgs/PidState.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "c2_hw/PidParamsConfig.h"

#include <boost/thread/mutex.hpp>
#include <boost/algorithm/clamp.hpp>

// Realtime buffer
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace c2_hw {

class Pid {
public:

	struct Gains {
		// Optional constructor for passing in values without antiwindup
		Gains(double p, double i, double d, double i_max, double i_min)
			: p_gain_(p),
			  i_gain_(i),
			  d_gain_(d),
			  i_max_(i_max),
			  i_min_(i_min),
			  antiwindup_(false)
		{}
		// Optional constructor for passing in values
		Gains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
			: p_gain_(p),
			  i_gain_(i),
			  d_gain_(d),
			  i_max_(i_max),
			  i_min_(i_min),
			  antiwindup_(antiwindup)
		{}
		// Default constructor
		Gains()
			: p_gain_(0.0),
			  i_gain_(0.0),
			  d_gain_(0.0),
			  i_max_(0.0),
			  i_min_(0.0),
			  antiwindup_(false)
		{}
		double p_gain_;   /**< Proportional gain. */
		double i_gain_;   /**< Integral gain. */
		double d_gain_;   /**< Derivative gain. */
		double i_max_;    /**< Maximum allowable integral term. */
		double i_min_;    /**< Minimum allowable integral term. */
		bool antiwindup_; /**< Antiwindup. */
	};

	/*!
	 * \brief Constructor, zeros out Pid values when created and
	 *        initialize Pid-gains and integral term limits.
	 *        Does not initialize dynamic reconfigure for PID gains
	 *
	 * \param p  The proportional gain.
	 * \param i  The integral gain.
	 * \param d  The derivative gain.
	 * \param i_max The max integral windup.
	 * \param i_min The min integral windup.
	 */
	Pid(double p = 0.0, double i = 0.0, double d = 0.0, double i_max = 0.0, double i_min = -0.0, bool antiwindup = false)
		: dynamic_reconfig_initialized_(false) {
		setGains(p,i,d,i_max,i_min,antiwindup);

		reset();
	}

	Pid(const Pid &source) : dynamic_reconfig_initialized_(false) {
		// Copy the realtime buffer to then new PID class
		gains_buffer_ = source.gains_buffer_;

		// Reset the state of this PID controller
		reset();
	}

	/*!
	 * \brief Destructor of Pid class.
	 */
	~Pid() {
	}

	/*!
	 * \brief Zeros out Pid values and initialize Pid-gains and integral term limits
	 *        Does not initialize dynamic reconfigure for PID gains
	 *
	 * \param p  The proportional gain.
	 * \param i  The integral gain.
	 * \param d  The derivative gain.
	 * \param i_max The max integral windup.
	 * \param i_min The min integral windup.
	 */
	void initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup = false) {
		setGains(p,i,d,i_max,i_min, antiwindup);

		reset();
	}

	/*!
	 * \brief Zeros out Pid values and initialize Pid-gains and integral term limits
	 *        Initializes dynamic reconfigure for PID gains
	 *
	 * \param p  The proportional gain.
	 * \param i  The integral gain.
	 * \param d  The derivative gain.
	 * \param i_max The max integral windup.
	 * \param i_min The min integral windup.
	 */
	void initPid(double p, double i, double d, double i_max, double i_min, const ros::NodeHandle& nh) {
		initPid(p, i, d, i_max, i_min);

		initDynamicReconfig(nh);
	}

	void initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup, const ros::NodeHandle& nh) {
		initPid(p, i, d, i_max, i_min, antiwindup);

		initDynamicReconfig(nh);
	}

	/*!
	 * \brief Initialize PID with the parameters in a namespace
	 *        Initializes dynamic reconfigure for PID gains
	 *
	 * \param prefix The namespace prefix.
	 * \param quiet If true, no error messages will be emitted on failure.
	 */
	bool initParam(const std::string& prefix, const bool quiet=false) {
		ros::NodeHandle nh(prefix);
		return init(nh, quiet);
	}

	/*!
	 * \brief Initialize PID with the parameters in a NodeHandle namespace
	 *        Initializes dynamic reconfigure for PID gains
	 *
	 * \param n The NodeHandle which should be used to query parameters.
	 * \param quiet If true, no error messages will be emitted on failure.
	 */
	bool init(const ros::NodeHandle &node, const bool quiet=false) {
		ros::NodeHandle nh(node);

		Gains gains;

		// Load PID gains from parameter server
		if (!nh.getParam("p", gains.p_gain_)) {
			if (!quiet) {
				ROS_ERROR("No p gain specified for pid.  Namespace: %s", nh.getNamespace().c_str());
			}
			return false;
		}
		// Only the P gain is required, the I and D gains are optional and default to 0:
		nh.param("i", gains.i_gain_, 0.0);
		nh.param("d", gains.d_gain_, 0.0);

		// Load integral clamp from param server or default to 0
		double i_clamp;
		nh.param("i_clamp", i_clamp, 0.0);
		gains.i_max_ = std::abs(i_clamp);
		gains.i_min_ = -std::abs(i_clamp);
		if(nh.hasParam("i_clamp_min")) {
			nh.param("i_clamp_min", gains.i_min_, gains.i_min_); // use i_clamp_min parameter, otherwise keep -i_clamp
			gains.i_min_ = -std::abs(gains.i_min_); // make sure the value is <= 0
		}
		if(nh.hasParam("i_clamp_max")) {
			nh.param("i_clamp_max", gains.i_max_, gains.i_max_); // use i_clamp_max parameter, otherwise keep i_clamp
			gains.i_max_ = std::abs(gains.i_max_); // make sure the value is >= 0
		}
		nh.param("antiwindup", gains.antiwindup_, false);

		nh.param("publish_state", publish_state_, false);

		if(publish_state_) {
			state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::PidState>());
			state_publisher_->init(nh, "state", 1);
		}

		setGains(gains);

		reset();
		initDynamicReconfig(nh);

		return true;
	}



	/**
	 * @brief Start the dynamic reconfigure node and load the default values
	 * @param node - a node handle where dynamic reconfigure services will be published
	 */
	void initDynamicReconfig(const ros::NodeHandle &node) {
		ROS_DEBUG_STREAM_NAMED("pid","Initializing dynamic reconfigure in namespace "
		                       << node.getNamespace());

		// Start dynamic reconfigure server
		param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
		dynamic_reconfig_initialized_ = true;

		// Set Dynamic Reconfigure's gains to Pid's values
		updateDynamicReconfig();

		// Set callback
		param_reconfig_callback_ = boost::bind(&Pid::dynamicReconfigCallback, this, _1, _2);
		param_reconfig_server_->setCallback(param_reconfig_callback_);
	}

	/*!
	 * \brief Reset the state of this PID controller
	 */
	void reset() {
		p_error_last_ = 0.0;
		p_error_ = 0.0;
		i_error_ = 0.0;
		d_error_ = 0.0;
		cmd_ = 0.0;
	}

	/*!
	 * \brief Get PID gains for the controller.
	 * \param p  The proportional gain.
	 * \param i  The integral gain.
	 * \param d  The derivative gain.
	 * \param i_max The max integral windup.
	 * \param i_min The min integral windup.
	 */
	void getGains(double &p, double &i, double &d, double &i_max, double &i_min) {
		bool antiwindup;
		getGains(p, i, d, i_max, i_min, antiwindup);
	}
	void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup) {
		Gains gains = *gains_buffer_.readFromRT();

		p     = gains.p_gain_;
		i     = gains.i_gain_;
		d     = gains.d_gain_;
		i_max = gains.i_max_;
		i_min = gains.i_min_;
		antiwindup = gains.antiwindup_;
	}

	/*!
	 * \brief Get PID gains for the controller.
	 * \return gains A struct of the PID gain values
	 */
	Gains getGains() {
		return *gains_buffer_.readFromRT();
	}

	/*!
	 * \brief Set PID gains for the controller.
	 * \param p  The proportional gain.
	 * \param i  The integral gain.
	 * \param d  The derivative gain.
	 * \param i_max The max integral windup.
	 * \param i_min The min integral windup.
	 */
	void setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup = false) {
		Gains gains(p,i,d,i_max,i_min, antiwindup);

		setGains(gains);
	}

	/*!
	 * \brief Set PID gains for the controller.
	 * \param gains A struct of the PID gain values
	 */
	void setGains(const Gains &gains) {
		gains_buffer_.writeFromNonRT(gains);

		// Update dynamic reconfigure with the new gains
		updateDynamicReconfig(gains);
	}

	/**
	 * @brief Set Dynamic Reconfigure's gains to Pid's values
	 */
	void updateDynamicReconfig() {
		// Make sure dynamic reconfigure is initialized
		if(!dynamic_reconfig_initialized_)
			return;

		// Get starting values
		PidParamsConfig config;

		// Get starting values
		getGains(config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min, config.antiwindup);

		updateDynamicReconfig(config);
	}

	void updateDynamicReconfig(Gains gains_config) {
		// Make sure dynamic reconfigure is initialized
		if(!dynamic_reconfig_initialized_)
			return;

		PidParamsConfig config;

		// Convert to dynamic reconfigure format
		config.p = gains_config.p_gain_;
		config.i = gains_config.i_gain_;
		config.d = gains_config.d_gain_;
		config.i_clamp_max = gains_config.i_max_;
		config.i_clamp_min = gains_config.i_min_;
		config.antiwindup = gains_config.antiwindup_;

		updateDynamicReconfig(config);
	}
	void updateDynamicReconfig(PidParamsConfig config) {
		// Make sure dynamic reconfigure is initialized
		if(!dynamic_reconfig_initialized_)
			return;

		// Set starting values, using a shared mutex with dynamic reconfig
		param_reconfig_mutex_.lock();
		param_reconfig_server_->updateConfig(config);
		param_reconfig_mutex_.unlock();
	}

	/**
	 * \brief Update the PID parameters from dynamics reconfigure
	 */
	void dynamicReconfigCallback(PidParamsConfig &config, uint32_t /*level*/) {
		ROS_DEBUG_STREAM_NAMED("pid","Dynamics reconfigure callback recieved.");

		// Set the gains
		setGains(config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min, config.antiwindup);
	}

	/*!
	 * \brief Set the PID error and compute the PID command with nonuniform time
	 * step size. The derivative error is computed from the change in the error
	 * and the timestep \c dt.
	 *
	 * \param error  Error since last call (error = target - state)
	 * \param dt Change in time since last call
	 *
	 * \returns PID command
	 */
	double computeCommand(double error, ros::Duration dt) {

		if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
			return 0.0;

		double error_dot = d_error_;

		// Calculate the derivative error
		if (dt.toSec() > 0.0) {
			error_dot = (error - p_error_last_) / dt.toSec();
			p_error_last_ = error;
		}

		return computeCommand(error, error_dot, dt);
	}

	/*!
	 * \brief Set the PID error and compute the PID command with nonuniform
	 * time step size. This also allows the user to pass in a precomputed
	 * derivative error.
	 *
	 * \param error Error since last call (error = target - state)
	 * \param error_dot d(Error)/dt since last call
	 * \param dt Change in time since last call
	 *
	 * \returns PID command
	 */
	double computeCommand(double error, double error_dot, ros::Duration dt) {
		// Get the gain parameters from the realtime buffer
		Gains gains = *gains_buffer_.readFromRT();

		double p_term, d_term, i_term;
		p_error_ = error; // this is error = target - state
		d_error_ = error_dot;

		if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
			return 0.0;

		// Calculate proportional contribution to command
		p_term = gains.p_gain_ * p_error_;

		// Calculate the integral of the position error
		i_error_ += dt.toSec() * p_error_;

		if(gains.antiwindup_) {
			// Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
			i_error_ = boost::algorithm::clamp(i_error_,
			                                   gains.i_min_ / std::abs(gains.i_gain_),
			                                   gains.i_max_ / std::abs(gains.i_gain_));
		}

		// Calculate integral contribution to command
		i_term = gains.i_gain_ * i_error_;

		if(!gains.antiwindup_) {
			// Limit i_term so that the limit is meaningful in the output
			i_term = boost::algorithm::clamp(i_term, gains.i_min_, gains.i_max_);
		}

		// Calculate derivative contribution to command
		d_term = gains.d_gain_ * d_error_;

		// Compute the command
		cmd_ = p_term + i_term + d_term;

		// Publish controller state if configured
		if (publish_state_ && state_publisher_) {
			if (state_publisher_->trylock()) {
				state_publisher_->msg_.header.stamp = ros::Time::now();
				state_publisher_->msg_.timestep = dt;
				state_publisher_->msg_.error = error;
				state_publisher_->msg_.error_dot = error_dot;
				state_publisher_->msg_.p_error = p_error_;
				state_publisher_->msg_.i_error = i_error_;
				state_publisher_->msg_.d_error = d_error_;
				state_publisher_->msg_.p_term = p_term;
				state_publisher_->msg_.i_term = i_term;
				state_publisher_->msg_.d_term = d_term;
				state_publisher_->msg_.i_max = gains.i_max_;
				state_publisher_->msg_.i_min = gains.i_min_;
				state_publisher_->msg_.output = cmd_;
				state_publisher_->unlockAndPublish();
			}
		}

		return cmd_;
	}


	/*!
	 * \brief Update the Pid loop with nonuniform time step size.
	 *
	 * \deprecated in ROS Hydro. This function assumes <tt> p_error = (state - target) </tt>
	 * which is an unconventional definition of the error. Please use \ref
	 * computeCommand instead, which assumes <tt> error = (target - state) </tt>. Note
	 * that calls to \ref computeCommand should not be mixed with calls to \ref
	 * updatePid.
	 *
	 * \param p_error  Error since last call (p_state-p_target)
	 * \param dt Change in time since last call
	 */
	ROS_DEPRECATED double updatePid(double error, ros::Duration dt) {
		return -computeCommand(error, dt);
	}

	/*!
	 * \brief Update the Pid loop with nonuniform time step size. This update
	 * call allows the user to pass in a precomputed derivative error.
	 *
	 * \deprecated in ROS Hydro. This function assumes <tt> p_error = (state - target) </tt>
	 * which is an unconventional definition of the error. Please use \ref
	 * computeCommand instead, which assumes <tt> error = (target - state) </tt>. Note
	 * that calls to \ref computeCommand should not be mixed with calls to \ref
	 * updatePid.
	 *
	 * \param error  Error since last call (p_state-p_target)
	 * \param error_dot d(Error)/dt since last call
	 * \param dt Change in time since last call
	 */
	ROS_DEPRECATED double updatePid(double error, double error_dot, ros::Duration dt) {
		return -computeCommand(error, error_dot, dt);
	}


	/*!
	 * \brief Set current command for this PID controller
	 */
	void setCurrentCmd(double cmd) {
		cmd_ = cmd;
	}

	/*!
	 * \brief Return current command for this PID controller
	 */
	double getCurrentCmd() {
		return cmd_;
	}

	/*!
	 * \brief Return PID error terms for the controller.
	 * \param pe  The proportional error.
	 * \param ie  The integral error.
	 * \param de  The derivative error.
	 */
	void getCurrentPIDErrors(double *pe, double *ie, double *de) {
		// Get the gain parameters from the realtime buffer
		Gains gains = *gains_buffer_.readFromRT();

		*pe = p_error_;
		*ie = i_error_;
		*de = d_error_;
	}


	/*!
	 * \brief Print to console the current parameters
	 */
	void printValues() {
		Gains gains = getGains();

		ROS_INFO_STREAM_NAMED("pid","Current Values of PID Class:\n"
		                      << "  P Gain: " << gains.p_gain_ << "\n"
		                      << "  I Gain: " << gains.i_gain_ << "\n"
		                      << "  D Gain: " << gains.d_gain_ << "\n"
		                      << "  I_Max:  " << gains.i_max_  << "\n"
		                      << "  I_Min:  " << gains.i_min_  << "\n"
		                      << "  Antiwindup:  " << gains.antiwindup_  << "\n"
		                      << "  P_Error_Last: " << p_error_last_  << "\n"
		                      << "  P_Error:      " << p_error_  << "\n"
		                      << "  I_Error:       " << i_error_  << "\n"
		                      << "  D_Error:      " << d_error_  << "\n"
		                      << "  Command:      " << cmd_
		                     );

	}

	/*!
	 * @brief Custom assignment operator
	 *        Does not initialize dynamic reconfigure for PID gains
	 */
	Pid &operator =(const Pid& source) {
		if (this == &source)
			return *this;

		// Copy the realtime buffer to then new PID class
		gains_buffer_ = source.gains_buffer_;

		// Reset the state of this PID controller
		reset();

		return *this;
	}

private:

	// Store the PID gains in a realtime buffer to allow dynamic reconfigure to update it without
	// blocking the realtime update loop
	realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

	boost::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::PidState> > state_publisher_;
	bool publish_state_;

	double p_error_last_; /**< _Save position state for derivative state calculation. */
	double p_error_; /**< Position error. */
	double i_error_; /**< Integral of position error. */
	double d_error_; /**< Derivative of position error. */
	double cmd_;     /**< Command to send. */

	// Dynamics reconfigure
	bool dynamic_reconfig_initialized_;
	typedef dynamic_reconfigure::Server<PidParamsConfig> DynamicReconfigServer;
	boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
	DynamicReconfigServer::CallbackType param_reconfig_callback_;

	boost::recursive_mutex param_reconfig_mutex_;

};

}

#endif
