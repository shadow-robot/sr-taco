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

    //TODO: add measurement model (specific to the sensor
    // will be different depending on if we're using a pose
    // or the saliency map)
    /*
    // create matrix H for linear measurement model
    Matrix H(1,2);
    double wall_ct = 2/(sqrt(pow(RICO_WALL,2.0) + 1));
    H = 0.0;
    H(1,1) = wall_ct * RICO_WALL;
    H(1,2) = 0 - wall_ct;

    // Construct the measurement noise (a scalar in this case)
    ColumnVector measNoise_Mu(1);
    measNoise_Mu(1) = MU_MEAS_NOISE;

    SymmetricMatrix measNoise_Cov(1);
    measNoise_Cov(1,1) = SIGMA_MEAS_NOISE;
    Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

    // create the model
    LinearAnalyticConditionalGaussian meas_pdf(H, measurement_Uncertainty);
    LinearAnalyticMeasurementModelGaussianUncertainty meas_model(&meas_pdf);
    */

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

  void PredictionModel::new_measurement(MatrixWrapper::ColumnVector measurement)
  {
    kalman_filter_->Update( system_model_.get(),
                            measurement_model_.get(),
                            measurement);

    //get the result
    //TODO: this could probably go in a different function called at a
    // regular interval
    posterior_.reset( kalman_filter_->PostGet() );
    ROS_ERROR_STREAM("Object is probably at: " << posterior_->ExpectedValueGet() << " \n"
                     << "  -> Covariance of: " << posterior_->CovarianceGet());
  }
}



/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
 */
