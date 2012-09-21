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

#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

    /**
     * regularly updates the model:
     *  disperses the object position based on the current
     *  linear twist of the object
     *
     * @return the current object position estimation with a covariance matrix
     */
    geometry_msgs::PoseWithCovarianceStamped update(double twist_x, double twist_y, double twist_z);

  protected:
    //the system model
    boost::shared_ptr<BFL::Gaussian> system_uncertainty_;
    boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> system_pdf_;
    boost::shared_ptr<BFL::LinearAnalyticSystemModelGaussianUncertainty> system_model_;

    //The measurement model
    boost::shared_ptr<BFL::Gaussian> measurement_uncertainty_;
    boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> measurement_pdf_;
    boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> measurement_model_;

    //The prior knowledge (-> where is the object at the beginning)
    boost::shared_ptr<BFL::Gaussian> prior_;

    ///The Kalman filter
    boost::shared_ptr<BFL::ExtendedKalmanFilter> kalman_filter_;

    ///The result
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior_;

    ///The result to be sent back to the ROS node
    geometry_msgs::PoseWithCovarianceStamped results_;

    ///Frequency at which the model is refreshed (as an inpact when updating the model due to the velocity)
    double refresh_frequency_;
  };
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
 */

#endif /* PREDICTION_MODEL_HPP_ */
