// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Unscented Kalman Filter
//
// ####### ANSWER FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 0.05;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 2;
constexpr double INIT_PSI_STD = 5.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

// ----------------------------------------------------------------------- //
// USEFUL HELPER FUNCTIONS
std::vector<VectorXd> generateSigmaPoints(VectorXd state, MatrixXd cov)
{
    std::vector<VectorXd> sigmaPoints;
    int numStates = state.size();
    double lambda = 3.0 - numStates;
    MatrixXd sqrtCov = cov.llt().matrixL();
    sigmaPoints.push_back(state);
    for(int iState = 0; iState < numStates; ++iState)
    {
        sigmaPoints.push_back(state + sqrt(lambda + numStates) * sqrtCov.col(iState));
        sigmaPoints.push_back(state - sqrt(lambda + numStates) * sqrtCov.col(iState));
    }
    return sigmaPoints;
}

std::vector<double> generateSigmaWeights(unsigned int numStates)
{
    std::vector<double> weights;
    double lambda = 3.0 - numStates;
    double w0 = lambda / (lambda+numStates);
    double wi = 0.5/(numStates+lambda);
    weights.push_back(w0);
    for(int i = 0; i < 2*numStates; ++i){weights.push_back(wi);}
    return weights;
}

VectorXd normaliseState(VectorXd state)
{
    state(2) = wrapAngle(state(2));
    return state;
}
VectorXd normaliseLidarMeasurement(VectorXd meas)
{
    meas(1) = wrapAngle(meas(1));
    return meas;
}

/*
VectorXd calculateMeanFromSigmaPoints(std::vector<VectorXd> sigma_points, double lambda, double n)
{



}
VectorXd calculateCovFromSigmaPoints(std::vector<VectorXd> sigma_points)
{

}
*/
// ----------------------------------------------------------------------- //

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
}

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the 
        // section below.
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // Hint: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1)
        {
            int n_x = state.size();
            int n_z = 2;

            // Measurement Vector
            VectorXd z = VectorXd::Zero(n_z);
            z << meas.range, meas.theta;

            // Noise Covariance Matrix
            MatrixXd R = MatrixXd::Zero(n_z,n_z);
            R(0,0) = LIDAR_RANGE_STD*LIDAR_RANGE_STD;
            R(1,1) = LIDAR_THETA_STD*LIDAR_THETA_STD;



            std::vector<VectorXd> x_sig = generateSigmaPoints(state, cov);
            std::vector<double> weights_sig = generateSigmaWeights(n_x);


            std::vector<VectorXd> z_sig;
            for (const auto& x : x_sig)
            {
                VectorXd z_hat = VectorXd::Zero(n_z);
                double delta_x = map_beacon.x - x[0];
                double delta_y = map_beacon.y - x[1];
                double zhat_range = sqrt(delta_x*delta_x + delta_y*delta_y);
                double zhat_theta = atan2(delta_y,delta_x) - x[2];
                z_hat << zhat_range, zhat_theta;
                z_sig.push_back(normaliseLidarMeasurement(z_hat));
            }

            // Calculate Measurement Mean
            VectorXd z_mean = VectorXd::Zero(n_z);
            for(unsigned int i = 0; i < z_sig.size(); ++i){z_mean += weights_sig[i] * z_sig[i];}

            // Calculate Innovation Covariance
            MatrixXd Py = MatrixXd::Zero(n_z,n_z);
            for(unsigned int i = 0; i < z_sig.size(); ++i)
            {
                VectorXd diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Py += weights_sig[i] * diff * diff.transpose();
            }
            Py = Py + R;

            MatrixXd Pxy = MatrixXd::Zero(n_x, n_z);
            for(unsigned int i = 0; i < 2*n_x + 1; ++i)
            {
                VectorXd x_diff = normaliseState(x_sig[i] - state);
                VectorXd z_diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Pxy += weights_sig[i] * x_diff * z_diff.transpose();
            }

            MatrixXd K = Pxy*Py.inverse();
            VectorXd y = normaliseLidarMeasurement(z - z_mean);

            state = state + K*y;
            cov = cov - K * Py * K.transpose();            

        }
        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the  
        // section below.
        // Hint: You can use the constants: ACCEL_STD, GYRO_STD
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE


        MatrixXd Q = Matrix2d::Zero();
        Q(0,0) = dt*GYRO_STD*GYRO_STD;
        Q(1,1) = dt*ACCEL_STD*ACCEL_STD;

        // Augment the State Vector with Noise States
        int n_x = state.size();
        int n_v = 2;
        int n_aug = n_x + n_v;

        VectorXd x_aug = VectorXd::Zero(n_aug);
        x_aug.head(n_x) = state;

        MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);
        P_aug.topLeftCorner(n_x,n_x) = cov;
        P_aug.bottomRightCorner(n_v,n_v) = Q;

        std::vector<VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug);
        std::vector<double> sigma_weights = generateSigmaWeights(n_aug);

        std::vector<VectorXd> sigma_points_predict;
        for (const auto& sigma_point : sigma_points)
        {
            double x = sigma_point(0);
            double y = sigma_point(1);
            double psi = sigma_point(2);
            double V = sigma_point(3);
            double psi_dot = gyro.psi_dot;
            double psi_dot_noise = sigma_point(4);
            double accel_noise = sigma_point(5);
            double x_new = x + dt * V * cos(psi);
            double y_new = y + dt * V * sin(psi);
            double psi_new = psi + dt * (psi_dot+psi_dot_noise);
            double V_new = V + dt * accel_noise;
            VectorXd sigma_point_predict = Vector4d();
            sigma_point_predict << x_new,y_new,psi_new,V_new;
            sigma_points_predict.push_back(normaliseState(sigma_point_predict));
        }


        // Calculate Mean
        state = VectorXd::Zero(n_x);
        for(unsigned int i = 0; i < sigma_points_predict.size(); ++i){state += sigma_weights[i] * sigma_points_predict[i];}

        // Calculate Covariance
        cov = MatrixXd::Zero(n_x,n_x);
        for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
        {
            VectorXd diff = normaliseState(sigma_points_predict[i] - state);
            cov += sigma_weights[i] * diff * diff.transpose();
        }

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    } 
}



void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2,4);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,meas.y;
        H << 1,0,0,0,0,1,0,0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        state = state + K*y;
        cov = (Matrix4d::Identity() - K*H) * cov;

        setState(state);
        setCovariance(cov);
    }
    else
    {
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state(0) = meas.x;
        state(1) = meas.y;
        cov(0,0) = GPS_POS_STD*GPS_POS_STD;
        cov(1,1) = GPS_POS_STD*GPS_POS_STD;
        cov(2,2) = INIT_PSI_STD*INIT_PSI_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

        setState(state);
        setCovariance(cov);
    }             
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0],state[1],state[2],state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}
