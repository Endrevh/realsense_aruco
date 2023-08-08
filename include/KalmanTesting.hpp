#ifndef KALMANTESTING_H
#define KALMANTESTING_H

#include "KalmanFilter.hpp"
#include <fstream>

using namespace std;

class KalmanTesting {
private:
    KalmanFilterSintef kalmanFilter_;
    vector<VectorXd> trueStateTrajectory;
    vector<VectorXd> estimatedStateTrajectory;
    vector<double> timeTrajectory;

    mt19937 randomGenerator;
    uniform_real_distribution<double> randomDistribution;

public:
    KalmanTesting(KalmanFilterSintef kalmanFilter);

    void runSimulation(int numSteps);

    const vector<VectorXd>& getTrueStateTrajectory() const {
        return trueStateTrajectory;
    }

    const vector<VectorXd>& getEstimatedStateTrajectory() const {
        return estimatedStateTrajectory;
    }

    void writeTrajectoriesToFile(const string& filename);
};

#endif