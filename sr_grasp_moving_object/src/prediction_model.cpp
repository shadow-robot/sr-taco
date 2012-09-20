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

#include <sr_grasp_moving_object/prediction_model.hpp>
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
const double PredictionModel::mu_noise_x_const_ = 0.0;
const double PredictionModel::mu_noise_y_const_ = 0.0;
const double PredictionModel::mu_noise_z_const_ = 0.0;

//TODO: select realistic noise value
const double PredictionModel::sigma_noise_x_const_ = 0.1;
const double PredictionModel::sigma_noise_y_const_ = 0.1;
const double PredictionModel::sigma_noise_z_const_ = 0.1;

const double PredictionModel::prior_mu_x_const_ = 0.0;
const double PredictionModel::prior_mu_y_const_ = 0.0;
const double PredictionModel::prior_mu_z_const_ = 0.0;

//TODO: choose sigmas so that the initial Gaussian covers the whole image
const double PredictionModel::prior_sigma_x_const_ = 150.0;
const double PredictionModel::prior_sigma_y_const_ = 150.0;
const double PredictionModel::prior_sigma_z_const_ = 150.0;

  PredictionModel::PredictionModel()
  {
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


    system_matrices_.reset(new std::vector<MatrixWrapper::Matrix>(2));
    (*system_matrices_.get())[0] = matrix_pos;
    (*system_matrices_.get())[0] = matrix_vel;

    MatrixWrapper::ColumnVector sys_noise_mu(3);
    sys_noise_mu(1) = mu_noise_x_const_;
    sys_noise_mu(2) = mu_noise_y_const_;
    sys_noise_mu(3) = mu_noise_z_const_;

    MatrixWrapper::SymmetricMatrix sys_noise_cov(3);
    sys_noise_cov = 0.0;
    sys_noise_cov(1,1) = sigma_noise_x_const_;
    sys_noise_cov(1,2) = 0.0;
    sys_noise_cov(1,3) = 0.0;
    sys_noise_cov(2,1) = 0.0;
    sys_noise_cov(2,2) = sigma_noise_y_const_;
    sys_noise_cov(2,3) = 0.0;
    sys_noise_cov(3,1) = 0.0;
    sys_noise_cov(3,2) = 0.0;
    sys_noise_cov(3,3) = sigma_noise_z_const_;

    system_uncertainty_.reset( new BFL::Gaussian(sys_noise_mu, sys_noise_cov) );

    system_pdf_.reset(new BFL::LinearAnalyticConditionalGaussian(matrix_pos, *system_uncertainty_.get()));
    system_model_.reset(new BFL::LinearAnalyticSystemModelGaussianUncertainty(system_pdf_.get()));


    //The measurement model and system model are the same in this case as we're getting
    // a 3d position for the object.
    measurement_pdf_.reset( new BFL::LinearAnalyticConditionalGaussian(matrix_pos, *system_uncertainty_.get()) );
    measurement_model_.reset(new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(measurement_pdf_.get()));

    //Initialising the prior knowledge
    // (we don't know where the object is so using a really flat
    // centered Gaussian)
    MatrixWrapper::ColumnVector prior_mu(3);
    prior_mu(1) = prior_mu_x_const_;
    prior_mu(2) = prior_mu_y_const_;
    prior_mu(3) = prior_mu_z_const_;
    MatrixWrapper::SymmetricMatrix prior_cov(3);
    prior_cov(1,1) = prior_sigma_x_const_;
    prior_cov(1,2) = 0.0;
    prior_cov(1,3) = 0.0;
    prior_cov(2,1) = 0.0;
    prior_cov(2,2) = prior_sigma_y_const_;
    prior_cov(2,3) = 0.0;
    prior_cov(3,1) = 0.0;
    prior_cov(3,2) = 0.0;
    prior_cov(3,3) = prior_sigma_z_const_;

    prior_.reset(new BFL::Gaussian(prior_mu, prior_cov));

    //finally initialising the filter
    kalman_filter_.reset(new BFL::ExtendedKalmanFilter(prior_.get()));
  };

  PredictionModel::~PredictionModel()
  {

  };

  void PredictionModel::new_measurement(double x, double y, double z)
  {

    MatrixWrapper::ColumnVector measurement(3);
    measurement(1) = x;
    measurement(2) = y;
    measurement(3) = z;

    ROS_DEBUG_STREAM("Updating filter with new measurement = " << x << " " << y << " " << z);

    kalman_filter_->Update( measurement_model_.get(),
                            measurement );
  }

  geometry_msgs::PoseWithCovarianceStamped PredictionModel::update( double twist_x, double twist_y, double twist_z )
  {
    //add system noise for dispersing the model if no value was received.
    // base the predicted movement of the model on the current linear twist
    MatrixWrapper::ColumnVector vel(3); vel = 0;
    vel(1) = twist_x;
    vel(2) = twist_y;
    vel(3) = twist_z;
    kalman_filter_->Update(system_model_.get(), vel);

    //get the result
    posterior_ = kalman_filter_->PostGet();
    ROS_DEBUG_STREAM("Object is probably at: " << posterior_->ExpectedValueGet() << " \n"
                     << "  -> Covariance of: " << posterior_->CovarianceGet());

    results_.pose.pose.position.x = posterior_->ExpectedValueGet()(1);
    results_.pose.pose.position.y = posterior_->ExpectedValueGet()(2);
    results_.pose.pose.position.z = posterior_->ExpectedValueGet()(3);

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
