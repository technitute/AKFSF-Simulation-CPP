// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Unscented Kalman Filter
//
// ####### ANSWER FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// ----------------------------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// ----------------------------------------------------------------------- //

// ----------------------------------------------------------------------- //
// USEFUL HELPER FUNCTIONS
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
std::vector<VectorXd> generateSigmaPoints(VectorXd state, MatrixXd cov)
{
    std::vector<VectorXd> sigmaPoints;

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE

    int numStates = state.size();
    double lambda = 3.0 - numStates;
    MatrixXd sqrtCov = cov.llt().matrixL();
    sigmaPoints.push_back(state);
    for(int iState = 0; iState < numStates; iState++)
    {
        sigmaPoints.push_back(state + sqrt(lambda + numStates) * sqrtCov.col(iState));
        sigmaPoints.push_back(state - sqrt(lambda + numStates) * sqrtCov.col(iState));
    }

    // ----------------------------------------------------------------------- //

    return sigmaPoints;
}

std::vector<double> generateSigmaWeights(unsigned int numStates)
{
    std::vector<double> weights;

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE
    
    double lambda = 3.0 - numStates;
    double w0 = lambda / (lambda+numStates);
    double wi = 0.5/(numStates+lambda);
    weights.push_back(w0);
    for(int i = 0; i < 2*numStates; ++i){weights.push_back(wi);}

    // ----------------------------------------------------------------------- //

    return weights;
}

VectorXd lidarMeasurementModel(VectorXd aug_state, double beaconX, double beaconY)
{
    VectorXd z_hat = VectorXd::Zero(2);

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE

    double x = aug_state(0);
    double y = aug_state(1);
    double psi = aug_state(2);
    double range_noise = aug_state(4);
    double theta_noise = aug_state(5);

    double delta_x = beaconX - x;
    double delta_y = beaconY - y;
    double zhat_range = sqrt(delta_x*delta_x + delta_y*delta_y) + range_noise;
    double zhat_theta = atan2(delta_y,delta_x) - aug_state[2] + theta_noise;
    z_hat << zhat_range, zhat_theta;

    // ----------------------------------------------------------------------- //

    return z_hat;
}

VectorXd vehicleProcessModel(VectorXd aug_state, double psi_dot, double dt)
{
    VectorXd new_state = VectorXd::Zero(4);

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE

    double x = aug_state(0);
    double y = aug_state(1);
    double psi = aug_state(2);
    double V = aug_state(3);
    double psi_dot_noise = aug_state(4);
    double accel_noise = aug_state(5);

    double x_new = x + dt * V * cos(psi);
    double y_new = y + dt * V * sin(psi);
    double psi_new = psi + dt * (psi_dot+psi_dot_noise);
    double V_new = V + dt * accel_noise;
    new_state << x_new,y_new,psi_new,V_new;

    // ----------------------------------------------------------------------- //

    return new_state;
}

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
        // HINT: Use the normaliseState() and normaliseLidarMeasurement() functions
        // to always keep angle values within correct range.
        // HINT: Do not normalise during sigma point calculation!
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1)
        {
            // Generate Measurement Vector
            VectorXd z = Vector2d::Zero();
            z << meas.range, meas.theta;

            // Generate Measurement Model Noise Covariance Matrix
            MatrixXd R = Matrix2d::Zero();
            R(0,0) = LIDAR_RANGE_STD*LIDAR_RANGE_STD;
            R(1,1) = LIDAR_THETA_STD*LIDAR_THETA_STD;

            // Augment the State Vector with Noise States
            int n_x = state.size();
            int n_v = 2;
            int n_z = 2;
            int n_aug = n_x + n_v;
            VectorXd x_aug = VectorXd::Zero(n_aug);
            MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);
            x_aug.head(n_x) = state;
            P_aug.topLeftCorner(n_x,n_x) = cov;
            P_aug.bottomRightCorner(n_v,n_v) = R;

            // Generate Augmented Sigma Points
            std::vector<VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug);
            std::vector<double> sigma_weights = generateSigmaWeights(n_aug);

            // Measurement Model Augmented Sigma Points
            std::vector<VectorXd> z_sig;
            for (const auto& sigma_point : sigma_points){z_sig.push_back(lidarMeasurementModel(sigma_point, map_beacon.x, map_beacon.y));}

            // Calculate Measurement Mean
            VectorXd z_mean = VectorXd::Zero(n_z);
            for(unsigned int i = 0; i < z_sig.size(); ++i){z_mean += sigma_weights[i] * z_sig[i];}

            // Calculate Innovation Covariance
            MatrixXd Py = MatrixXd::Zero(n_z,n_z);
            for(unsigned int i = 0; i < z_sig.size(); ++i)
            {
                VectorXd diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Py += sigma_weights[i] * diff * diff.transpose();
            }

            // Calculate Cross Covariance
            MatrixXd Pxy = MatrixXd::Zero(n_x, n_z);
            for(unsigned int i = 0; i < 2*n_x + 1; ++i)
            {
                VectorXd x_diff = normaliseState(sigma_points[i].head(n_x) - state);
                VectorXd z_diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Pxy += sigma_weights[i] * x_diff * z_diff.transpose();
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
        // HINT: Assume the state vector has the form [PX, PY, PSI, V].
        // HINT: Use the Gyroscope measurement as an input into the prediction step.
        // HINT: You can use the constants: ACCEL_STD, GYRO_STD
        // HINT: Use the normaliseState() function to always keep angle values within correct range.
        // HINT: Do NOT normalise during sigma point calculation!
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        // Generate Q Matrix
        MatrixXd Q = Matrix2d::Zero();
        Q(0,0) = GYRO_STD*GYRO_STD;
        Q(1,1) = ACCEL_STD*ACCEL_STD;

        // Augment the State Vector with Noise States
        int n_x = state.size();
        int n_w = 2;
        int n_aug = n_x + n_w;
        VectorXd x_aug = VectorXd::Zero(n_aug);
        MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);
        x_aug.head(n_x) = state;
        P_aug.topLeftCorner(n_x,n_x) = cov;
        P_aug.bottomRightCorner(n_w,n_w) = Q;

        // Generate Augmented Sigma Points
        std::vector<VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug);
        std::vector<double> sigma_weights = generateSigmaWeights(n_aug);

        // Predict Augmented Sigma Points
        std::vector<VectorXd> sigma_points_predict;
        for (const auto& sigma_point : sigma_points)
        {
            sigma_points_predict.push_back(vehicleProcessModel(sigma_point, gyro.psi_dot, dt));
        }

        // Calculate Mean
        state = VectorXd::Zero(n_x);
        for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
        {
            state += sigma_weights[i] * sigma_points_predict[i];
        }
        state = normaliseState(state);

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
    // All this code is the same as the LKF as the measurement model is linear
    // so the UKF update state would just produce the same result.
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