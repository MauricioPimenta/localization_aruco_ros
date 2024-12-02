/**
 * @file kalman_filter.h
 * @author Mauricio Bittencourt Pimenta
 * @brief Implementation of a generic Kalman Filter algorithm, based on the Table 3.1 of the book "Probabilistic Robotics" by Sebastian Thrun, Wolfram Burgard, and Dieter Fox.
 * @version 0.1
 * @date 2024-04-23
 */
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "matplotlibcpp.h"

#include <iostream>
#include <string>
#include <list>
#include <array>
#include <vector>

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm> // to use std::sort
#include <cmath>    // to use M_PI, std::sin and std::cos

#include <eigen3/Eigen/Eigen>


/**----------------------------------------------------------------------
 * * KalmanFilter Class
 *
 *  Probabilistic Linear Kalman Filter implementation
 *-----------------------------------------------------------------------**/
/**----------------------------------------------
 * *                   INFO
 *   The Kalman filter represents beliefs by the moments parameterization:
 *   At time t, the belief is represented by the mean mi_t and the covariance matrix Sigma_t.
 *
 **   Posteriors are Gaussian if the following three properties hold, in addition to the Markov assumption of the Bayes filter:
 **      1. The state transition probability p(x_t | u_t, x_(t-1)) must be a linear function in its arguments with added Gaussian noise.
    *        This is expressed by yhe following equation:
 **          x_t = A*x_(t-1) + B*u_t + e_t     where e_t ~ N(0, R) and R is the covariance matrix of the noise.
 *
 **      2. The sensor model p(z_t | x_t) must be a linear function in its arguments with added Gaussian noise.
    *          This is expressed by the following equation:
    *            z_t = C*x_t + delta_t     where delta_t ~ N(0, Q) and Q is the covariance matrix of the noise.
    *
 **      3. The initial belief must be Gaussian:
    *          u_0 ~ N(mi_0, Sigma_0)       where mi_0 is the mean and Sigma_0 is the covariance matrix.
 *
    *          bel(x_0) = p(x_0) = N(x_0 | mi_0, Sigma_0) = det(2*pi*Sigma_0)^(-1/2) * exp(-1/2 * (x_0 - mi_0)^T * Sigma_0^(-1) * (x_0 - mi_0))
    *
    *---------------------------------------------**/
class KalmanFilter
{
public:

    // Belief Structure - States Mean and Covariance Matrix
    typedef struct belief
    {
        // Mean Vector and Covariance Matrix
        Eigen::VectorXd State_Mean;  // Mean Vector - Vetor de Medias
        Eigen::MatrixXd State_Covariance;  // Covariance Matrix - Matriz de Covariancia
    } bel;


    /* Public Atributes */

        // Vetores do Sistema Linear em Espaço de Estados
        Eigen::VectorXd States;  // State Vector - Vetor de Estados
        Eigen::VectorXd Control;  // Control Vector - Vetor de Controle
        Eigen::VectorXd Measures;  // Measurement Vector - Vetor de Medidas - Zt

        // Matrizes da representação do Sistema Linear em Espaço de Estados
        Eigen::MatrixXd A;  // State Matrix - Matriz de Estados
        Eigen::MatrixXd B;  // Input or Control Matrix - Matriz de Controle

        Eigen::MatrixXd C;  // Output Matrix - Matriz de Saida
        Eigen::MatrixXd D;  // Feedthrough or Feedforward Matrix - Matriz de Transmissão Direta

        // Incertezas - Covariancias
        Eigen::MatrixXd R;  // State Covariance Matrix
        Eigen::MatrixXd Q;  // Measurement Covariance Matrix

        // Initial Belief and Last Belief
        bel Initial_Belief;
        bel Current_Belief;

        // Array of Beliefs in time
        std::vector<bel> Beliefs;


    /* Public Methods */

        /* CONSTRUCTORS */
        KalmanFilter(Eigen::VectorXd States, Eigen::VectorXd Control, Eigen::VectorXd Measures, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D, Eigen::MatrixXd R, Eigen::MatrixXd Q, Eigen::VectorXd Initial_States, Eigen::MatrixXd Initial_Covariance_Matrix)
        {
            this->States = States;
            this->Control = Control;
            this->Measures = Measures;
            this->A = A;
            this->B = B;
            this->C = C;
            this->D = D;
            this->R = R;
            this->Q = Q;
            this->Initial_Belief.State_Mean = Initial_States;
            this->Initial_Belief.State_Covariance = Initial_Covariance_Matrix;

            // Save the first belief in the array of beliefs
            this->Beliefs.push_back(this->Initial_Belief);

		}

        /* DESTRUCTOR */
        ~KalmanFilter();


        // Algorithm Kalman_filter Function
        // Inputs: Previous States (means vector and covariance matrix), Control Vector and measurement vector
        // Outputs: Updated States (means vector and covariance matrix)
        void Update_Beliefs(bel Previous_Belief, Eigen::VectorXd Control, Eigen::VectorXd Measures)
        {
            // Prediction Step
            // Predicted State Mean
            Eigen::VectorXd Predicted_State_Mean = A * Previous_Belief.State_Mean + B * Control;
            // Predicted State Covariance
            Eigen::MatrixXd Predicted_State_Covariance = A * Previous_Belief.State_Covariance * A.transpose() + R;

            // Update Step
            // Kalman Gain
            Eigen::MatrixXd Kalman_Gain = Predicted_State_Covariance * C.transpose() * (C * Predicted_State_Covariance * C.transpose() + Q).inverse();
            // Updated State Mean
            Eigen::VectorXd Updated_State_Mean = Predicted_State_Mean + Kalman_Gain * (Measures - C * Predicted_State_Mean);
            // Updated State Covariance
            Eigen::MatrixXd Updated_State_Covariance = (Eigen::MatrixXd::Identity(States.size(), States.size()) - Kalman_Gain * C) * Predicted_State_Covariance;

            // Return Updated Belief
            bel Updated_Belief;
            Updated_Belief.State_Mean = Updated_State_Mean;
            Updated_Belief.State_Covariance = Updated_State_Covariance;

            this->Current_Belief = Updated_Belief;
            this->Beliefs.push_back(Updated_Belief);
        }


private:
    /* Private Atributes */

    /* Private Methods */

};

#include <vector>
#include <map>
#include <string>

class ExtendedKalmanFilter {
    public:
        // Attributes //

        
        std::vector<Eigen::VectorXd> states;
        std::vector<Eigen::VectorXd> statesMeasurement;
        Eigen::Matrix3d sigma;
        Eigen::Matrix3d R;
        Eigen::Matrix3d Q;
        Eigen::Matrix3d G;
        Eigen::Matrix3d H;
        Eigen::Matrix3d K;
        double lastTimestamp;
        std::map<int, Eigen::Vector2d> landmarkLocations;
        std::map<int, int> landmarkIndexes;

        // Methods //

        /**------------------------------------------------------------------------------------------------
         *!                                        Constructor
            * @brief Constructor of the ExtendedKalmanFilter class
            *   @param dataset - The dataset to be used
            *   @param end_frame - The last frame of the dataset
            *   @param R - The covariance matrix of the noise of the state transition
            *   @param Q - The covariance matrix of the noise of the sensor model
            * @return None
            *
            * @details  This constructor initializes the ExtendedKalmanFilter class with the dataset, the
            *           last frame of the dataset, the covariance matrix of the noise of the state
            *           transition, and the covariance matrix of the noise of the sensor model.
            *
         *------------------------------------------------------------------------------------------------**/
        ExtendedKalmanFilter(const std::string& dataset, int end_frame, const Eigen::Matrix3d& R, const Eigen::Matrix3d& Q){
            initialization(R, Q);
        }

        void initialization(const Eigen::Matrix3d& R, const Eigen::Matrix3d& Q);
        void motionUpdate(const Eigen::VectorXd& control);
        void measurementUpdate(const Eigen::VectorXd& measurement);


};

#endif // EXTENDEDKALMANFILTER_H
