#include "KalmanTesting.hpp"


KalmanTesting::KalmanTesting(KalmanFilterSintef kalmanFilter)
    : randomDistribution(-0.05, 0.05), kalmanFilter_(kalmanFilter)
{
}

void KalmanTesting::runSimulation(int numSteps)
{

    trueStateTrajectory.clear();
    estimatedStateTrajectory.clear();

    double dt = kalmanFilter_.getDt();

    double sinusoidal_freq = 0.5;
    double acc_amplitude = 5.0;

    for (int step = 0; step < numSteps; ++step) {
        // Generate true acceleration (sine wave)

        double sine_arg = 2*M_PI*sinusoidal_freq;

        double trueAcceleration_x = acc_amplitude * sin(sine_arg*step*dt);
        double trueAcceleration_y = acc_amplitude * sin(sine_arg*step*dt + 2*M_PI/3);
        double trueAcceleration_z = acc_amplitude * sin(sine_arg*step*dt + 4*M_PI/3);

        double trueVelocity_x = -acc_amplitude * cos(sine_arg*step*dt) / (sine_arg);
        double trueVelocity_y = -acc_amplitude * cos(sine_arg*step*dt + 2*M_PI/3) / (sine_arg);
        double trueVelocity_z = -acc_amplitude * cos(sine_arg*step*dt + 4*M_PI/3) / (sine_arg);

        double truePosition_x = -acc_amplitude * sin(sine_arg*step*dt) / pow(sine_arg, 2);
        double truePosition_y = -acc_amplitude * sin(sine_arg*step*dt + 2*M_PI/3) / pow(sine_arg, 2);
        double truePosition_z = -acc_amplitude * sin(sine_arg*step*dt + 4*M_PI/3) / pow(sine_arg, 2);

        VectorXd trueStateEntry(6);
        trueStateEntry << truePosition_x, truePosition_y, truePosition_z, trueVelocity_x, trueVelocity_y, trueVelocity_z;

        // add noise to the first three states
        for (int i = 0; i < 3; ++i) {
            trueStateEntry[i] += randomDistribution(randomGenerator);
        }

        Vector3d z(trueStateEntry[0], trueStateEntry[1], trueStateEntry[2]);

        // Perform Kalman filter steps
        if (step == 0) 
        {
            kalmanFilter_.reset(z);
        }
        else
        {
            kalmanFilter_.predict();
            kalmanFilter_.correction(z);
        }

        VectorXd estimatedState = kalmanFilter_.getState();

        // Store values for analysis
        timeTrajectory.push_back(step*dt);
        trueStateTrajectory.push_back(trueStateEntry);
        estimatedStateTrajectory.push_back(estimatedState);
    }
}

void KalmanTesting::writeTrajectoriesToFile(const string& filename)
{
    ofstream file(filename);

    file.clear();

    file << "time,truePosition_x,truePosition_y,truePosition_z,trueVelocity_x,trueVelocity_y,trueVelocity_z,estimatedPosition_x,estimatedPosition_y,estimatedPosition_z,estimatedVelocity_x,estimatedVelocity_y,estimatedVelocity_z" << endl;

    for (int i = 0; i < timeTrajectory.size(); ++i) {
        file << timeTrajectory[i] << "," << trueStateTrajectory[i][0] << "," << trueStateTrajectory[i][1] << "," << trueStateTrajectory[i][2] << "," <<
            trueStateTrajectory[i][3] << "," << trueStateTrajectory[i][4] << "," << trueStateTrajectory[i][5] << "," <<
            estimatedStateTrajectory[i][0] << "," << estimatedStateTrajectory[i][1] << "," << estimatedStateTrajectory[i][2] << "," << 
            estimatedStateTrajectory[i][3] << "," << estimatedStateTrajectory[i][4] << "," << estimatedStateTrajectory[i][5] << endl;
    }

    file.close();
}