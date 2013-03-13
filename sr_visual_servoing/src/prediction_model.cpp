/**
 * @file   prediction_model.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <software@shadowrobot.com>
 * @date   Sep 13, 2012 11:08:40 AM 2012
 *
 *
 * Copyright 2011 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @brief This is the prediction model used to refine the position of the tracked object.
 *
 */

#include <sr_visual_servoing/prediction_model.hpp>
#include <ros/ros.h>

/**
 * Our model:
 *  x_k = x_{k-1} + twist_x{k-1} * delta_t
 *  y_k = y_{k-1} + twist_y{k-1} * delta_t
 *  z_k = z_{k-1} + twist_z{k-1} * delta_t
 *
 */

namespace sr_taco
{
  PredictionModel::PredictionModel()
    : nh_("~"), measurement_(3), meas_used_(true)
  {
    reset_model_();

    timer_ = nh_.createTimer(ros::Duration(0.1), &PredictionModel::reset, this, true);
  }

  PredictionModel::~PredictionModel()
  {
  };

  void PredictionModel::reset(const ros::TimerEvent&)
  {
    reset_model_();
  }

  void PredictionModel::reset_model_()
  {
    nh_.param<double>("refresh_frequency", refresh_frequency_, 100.0);

    MatrixWrapper::Matrix matrix_pos(3,3);
    matrix_pos(1,1) = 1.0;
    matrix_pos(1,2) = 0.0;
    matrix_pos(1,3) = 0.0;
    matrix_pos(2,1) = 0.0;
    matrix_pos(2,2) = 1.0;
    matrix_pos(2,3) = 0.0;
    matrix_pos(3,1) = 0.0;
    matrix_pos(3,2) = 0.0;
    matrix_pos(3,3) = 1.0;

    MatrixWrapper::Matrix matrix_vel(3,3);
    matrix_vel(1,1) = 1.0;
    matrix_vel(1,2) = 0.0;
    matrix_vel(1,3) = 0.0;
    matrix_vel(2,1) = 0.0;
    matrix_vel(2,2) = 1.0;
    matrix_vel(2,3) = 0.0;
    matrix_vel(3,1) = 0.0;
    matrix_vel(3,2) = 0.0;
    matrix_vel(3,3) = 1.0;

    std::vector<MatrixWrapper::Matrix> system_matrices(2);
    system_matrices[0] = matrix_pos;
    system_matrices[1] = matrix_vel;

    MatrixWrapper::ColumnVector sys_noise_mu(3);
    sys_noise_mu(1) = 0.0;
    sys_noise_mu(2) = 0.0;
    sys_noise_mu(3) = 0.0;

    // the std dev is dependent on the system
    //  refresh rate (otherwise, the faster we
    //  update the system, the more dispersed the
    //  data would be)
    double param;
    MatrixWrapper::SymmetricMatrix sys_noise_cov(3);
    sys_noise_cov = 0.0;
    nh_.param<double>("prediction_model/system/sigma/x", param, 0.2);
    sys_noise_cov(1,1) = param / refresh_frequency_;
    sys_noise_cov(1,2) = 0.0;
    sys_noise_cov(1,3) = 0.0;
    sys_noise_cov(2,1) = 0.0;
    nh_.param<double>("prediction_model/system/sigma/y", param, 0.2);
    sys_noise_cov(2,2) = param / refresh_frequency_;
    sys_noise_cov(2,3) = 0.0;
    sys_noise_cov(3,1) = 0.0;
    sys_noise_cov(3,2) = 0.0;
    nh_.param<double>("prediction_model/system/sigma/z", param, 0.2);
    sys_noise_cov(3,3) = param / refresh_frequency_;

    system_uncertainty_.reset( new BFL::Gaussian(sys_noise_mu, sys_noise_cov) );

    system_pdf_.reset(new BFL::LinearAnalyticConditionalGaussian(system_matrices, *system_uncertainty_.get()));
    system_model_.reset(new BFL::LinearAnalyticSystemModelGaussianUncertainty(system_pdf_.get()));


    //The measurement model and system model are the same in this case as we're getting
    // a 3d position for the object.
    MatrixWrapper::ColumnVector meas_noise_mu(3);
    sys_noise_mu(1) = 0.0;
    sys_noise_mu(2) = 0.0;
    sys_noise_mu(3) = 0.0;

    MatrixWrapper::SymmetricMatrix meas_noise_cov(3);
    meas_noise_cov = 0.0;
    nh_.param<double>("prediction_model/measurement/sigma/x", param, 0.2);
    meas_noise_cov(1,1) = param;
    meas_noise_cov(1,2) = 0.0;
    meas_noise_cov(1,3) = 0.0;
    meas_noise_cov(2,1) = 0.0;
    nh_.param<double>("prediction_model/measurement/sigma/y", param, 0.2);
    meas_noise_cov(2,2) = param;
    meas_noise_cov(2,3) = 0.0;
    meas_noise_cov(3,1) = 0.0;
    meas_noise_cov(3,2) = 0.0;
    nh_.param<double>("prediction_model/measurement/sigma/z", param, 0.2);
    meas_noise_cov(3,3) = param;

    measurement_uncertainty_.reset( new BFL::Gaussian(meas_noise_mu, meas_noise_cov) );

    measurement_pdf_.reset( new BFL::LinearAnalyticConditionalGaussian(matrix_pos, *measurement_uncertainty_.get()) );
    measurement_model_.reset(new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(measurement_pdf_.get()));

    //Initialising the prior knowledge
    // (we don't know where the object is so using a really flat
    // centered Gaussian)
    MatrixWrapper::ColumnVector prior_mu(3);
    nh_.param<double>("prediction_model/prior/mu/x", param, 0.0);
    prior_mu(1) = param;
    nh_.param<double>("prediction_model/prior/mu/y", param, 0.0);
    prior_mu(2) = param;
    nh_.param<double>("prediction_model/prior/mu/z", param, 0.0);
    prior_mu(3) = param;

    MatrixWrapper::SymmetricMatrix prior_cov(3);
    nh_.param<double>("prediction_model/prior/sigma/x", param, 50.0);
    prior_cov(1,1) = param;
    prior_cov(1,2) = 0.0;
    prior_cov(1,3) = 0.0;
    prior_cov(2,1) = 0.0;
    nh_.param<double>("prediction_model/prior/sigma/y", param, 50.0);
    prior_cov(2,2) = param;
    prior_cov(2,3) = 0.0;
    prior_cov(3,1) = 0.0;
    prior_cov(3,2) = 0.0;
    nh_.param<double>("prediction_model/prior/sigma/z", param, 50.0);
    prior_cov(3,3) = param;

    prior_.reset(new BFL::Gaussian(prior_mu, prior_cov));

    //finally initialising the filter
    kalman_filter_.reset(new BFL::ExtendedKalmanFilter(prior_.get()));
  }

  void PredictionModel::new_measurement(double x, double y, double z)
  {
    if( (x != x) | (y != y) | (z != z) )
    {
      ROS_WARN_STREAM("We received NaN, ignoring the new measurement: " << x << " " << y << " " << z );
      return;
    }

    measurement_(1) = x;
    measurement_(2) = y;
    measurement_(3) = z;

    //we have received a new measurement
    meas_used_ = false;
  }

  geometry_msgs::PoseWithCovarianceStamped PredictionModel::update( double twist_x, double twist_y, double twist_z )
  {
    //add system noise for dispersing the model if no value was received.
    // base the predicted movement of the model on the current linear twist
    // the velocity is divided by the refresh frequency as the velocity
    // is expressed in m.s-1
    MatrixWrapper::ColumnVector vel(3); vel = 0;
    vel(1) = twist_x / refresh_frequency_;
    vel(2) = twist_y / refresh_frequency_;
    vel(3) = twist_z / refresh_frequency_;

    //we have a new measurement waiting to be processed
    if( !meas_used_ )
    {
      ROS_DEBUG_STREAM("New measurement = " << measurement_ << " / vel: " <<  vel);

      kalman_filter_->Update( system_model_.get(), vel,
                              measurement_model_.get(),
                              measurement_);
      meas_used_ = true;
    }
    else
    {
      //no measurement only dispersion
      ROS_DEBUG_STREAM("Dispersion model, velocities: " << vel);
      kalman_filter_->Update(system_model_.get(), vel);
    }

    //get the result
    posterior_ = kalman_filter_->PostGet();
    ROS_DEBUG_STREAM("Object is probably at: " << posterior_->ExpectedValueGet() << " \n"
                     << "  -> Covariance of: " << posterior_->CovarianceGet());

    results_.pose.pose.position.x = posterior_->ExpectedValueGet()(1);
    results_.pose.pose.position.y = posterior_->ExpectedValueGet()(2);
    results_.pose.pose.position.z = posterior_->ExpectedValueGet()(3);

    double x, y, z;
    x = posterior_->ExpectedValueGet()(1);
    y = posterior_->ExpectedValueGet()(2);
    z = posterior_->ExpectedValueGet()(3);
    if( (x != x) | (y != y) | (z != z) )
    {
      ROS_WARN_STREAM("There was a problem, we predicted some NaN: " << x << " " << y << " " << z <<" / resetting filter." );
      reset_model_();
      timer_ = nh_.createTimer(ros::Duration(0.1), &PredictionModel::reset, this, true);
      //hack to mark it as failed
      results_.header.seq = 0;
      return results_;
    }
    else
    {
      //hack to mark it as success
      results_.header.seq = 1;
    }

    //TODO compute orientation as well?
    results_.pose.pose.orientation.x = 0.0;
    results_.pose.pose.orientation.y = 0.0;
    results_.pose.pose.orientation.z = 0.0;
    results_.pose.pose.orientation.w = 1.0;

    //The covariance from the pose is a 6x6 matrix
    // but we've only got a 3x3 matrix as we're estimating the position only
    results_.pose.covariance[0] = posterior_->CovarianceGet()(1,1);
    results_.pose.covariance[1] = posterior_->CovarianceGet()(1,2);
    results_.pose.covariance[2] = posterior_->CovarianceGet()(1,3);
    results_.pose.covariance[6] = posterior_->CovarianceGet()(2,1);
    results_.pose.covariance[7] = posterior_->CovarianceGet()(2,2);
    results_.pose.covariance[8] = posterior_->CovarianceGet()(2,3);
    results_.pose.covariance[12] = posterior_->CovarianceGet()(3,1);
    results_.pose.covariance[13] = posterior_->CovarianceGet()(3,2);
    results_.pose.covariance[14] = posterior_->CovarianceGet()(3,3);

    return results_;
  }
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
 */
