/**
 * @file   prediction_model.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <software@shadowrobot.com>
 * @date   Sep 12, 2012 11:30:01 AM 2012
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

#ifndef PREDICTION_MODEL_HPP_
#define PREDICTION_MODEL_HPP_


// bayesian filtering
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

#include <boost/smart_ptr.hpp>

/**
 * Our model:
 *  x_k = x_{k-1} + twist_x{k-1} * delta_t
 *  y_k = y_{k-1} + twist_y{k-1} * delta_t
 *  z_k = z_{k-1} + twist_z{k-1} * delta_t
 *
 */

namespace sr_taco
{
class PredictionModel
{
public:
  PredictionModel();
  ~PredictionModel();

  void new_measurement(double x, double y, double z);

protected:
  boost::shared_ptr<std::vector<MatrixWrapper::Matrix> > system_matrices_;

  //the system model
  boost::shared_ptr<BFL::Gaussian> system_uncertainty_;
  ///average noise around x, y and z
  static const double mu_noise_x_const_;
  static const double mu_noise_y_const_;
  static const double mu_noise_z_const_;

  ///standard deviation of the noise around x, y and z
  static const double sigma_noise_x_const_;
  static const double sigma_noise_y_const_;
  static const double sigma_noise_z_const_;

  //The model of the system (proba of the object being somewhere)
  boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> system_pdf_;
  boost::shared_ptr<BFL::LinearAnalyticSystemModelGaussianUncertainty> system_model_;

  //The measurement model
  boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> measurement_pdf_;
  boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> measurement_model_;

  //The prior knowledge (-> where is the object at the beginning)
  boost::shared_ptr<BFL::Gaussian> prior_;
  ///center of prior knowledge
  static const double prior_mu_x_const_;
  static const double prior_mu_y_const_;
  static const double prior_mu_z_const_;

  ///standard deviation of prior knowledge
  static const double prior_sigma_x_const_;
  static const double prior_sigma_y_const_;
  static const double prior_sigma_z_const_;

  ///The Kalman filter
  boost::shared_ptr<BFL::ExtendedKalmanFilter> kalman_filter_;

  ///The result
  BFL::Pdf<MatrixWrapper::ColumnVector>* posterior_;
};
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
 */

#endif /* PREDICTION_MODEL_HPP_ */
