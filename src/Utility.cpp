#include "Utility.hpp"

// Function to read CSV data into a 2D vector
vector<vector<float>> readCSV(const string& filename) {
    ifstream file(filename);
    vector<vector<float>> data;
    string line;

    while (getline(file, line)) {
        vector<float> row;
        stringstream lineStream(line);
        string cell;

        while (getline(lineStream, cell, ',')) {
            row.push_back(stof(cell));
        }

        data.push_back(row);
    }

    return data;
}

// Function to convert a 2D vector to cv::Mat
Mat vectorToMat(const vector<vector<float>>& data) {
    Mat mat(data.size(), data[0].size(), CV_64F);
    for (int i = 0; i < data.size(); ++i) {
        for (int j = 0; j < data[i].size(); ++j) {
            mat.at<float>(i, j) = data[i][j];
        }
    }
    return mat;
}

// Function to read camera matrix and distortion vector from CSV files and return as cv::Mat
void readCameraParameters(const string& cameraMatrixFile, const string& distortionVectorFile, Mat& camera_matrix, Mat& distortion_vector) {
    vector<vector<float>> camera_matrix_data = readCSV(cameraMatrixFile);
    vector<vector<float>> distortion_vector_data = readCSV(distortionVectorFile);

    // Assuming the camera matrix is a 3x3 matrix and the distortion vector has a size of 5
    if (camera_matrix_data.size() == 3 && camera_matrix_data[0].size() == 3 &&
        distortion_vector_data.size() == 1 && distortion_vector_data[0].size() == 5) {
        // Convert the data to cv::Mat
        camera_matrix = vectorToMat(camera_matrix_data).reshape(1);
        distortion_vector = vectorToMat(distortion_vector_data);
        distortion_vector.reshape(5,1);

    } else {
        cerr << "Invalid CSV files. Expected a 3x3 camera matrix and a 5-element distortion vector." << endl;
    }

    if (!camera_matrix.empty() && !distortion_vector.empty()) {
        // Now you have the camera matrix and distortion vector as cv::Mat
        // You can use them in your C++ code for further processing
        cout << "Camera Matrix:\n" << camera_matrix << endl;
        cout << "Distortion Vector:\n" << distortion_vector << endl;
    }
}

Matrix3d axisAngleToRotationMatrix(const Vector3d& axis_angle) {
    Matrix3d rotation_matrix;

    // Extract the axis and angle from the input vector
    Vector3d axis = axis_angle.normalized(); // Make sure the axis is a unit vector
    double angle = axis_angle.norm();

    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
    double one_minus_cos = 1.0 - cos_angle;

    // Skew-symmetric matrix associated with the axis
    Matrix3d K;
    K << 0.0, -axis(2), axis(1),
         axis(2), 0.0, -axis(0),
         -axis(1), axis(0), 0.0;

    rotation_matrix = Matrix3d::Identity() +
                      sin_angle * K +
                      one_minus_cos * K * K;

    return rotation_matrix;
}

Matrix3d rollPitchYawToRotationMatrix(double roll, double pitch, double yaw) {
    // Calculate the individual rotation matrices
    Matrix3d roll_matrix;
    roll_matrix << 1, 0, 0,
                   0, cos(roll), -sin(roll),
                   0, sin(roll), cos(roll);

    Matrix3d pitch_matrix;
    pitch_matrix << cos(pitch), 0, sin(pitch),
                    0, 1, 0,
                    -sin(pitch), 0, cos(pitch);

    Matrix3d yaw_matrix;
    yaw_matrix << cos(yaw), -sin(yaw), 0,
                  sin(yaw), cos(yaw), 0,
                  0, 0, 1;

    // Combine the rotation matrices in the order: roll -> pitch -> yaw
    Matrix3d rotation_matrix = roll_matrix * pitch_matrix * yaw_matrix;

    return rotation_matrix;
}

Matrix4d buildTransformationMatrix(Matrix3d rotationMatrix, Vector3d translationVector)
{
    Matrix4d transformationMatrix = Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix.block<3, 1>(0, 3) = translationVector;

    return transformationMatrix;
}

Vector3d transformVector(const Matrix4d& T_a_b, const Vector3d& vector_b) {
    // Homogeneous coordinates for vector_b
    Vector4d vector_b_homogeneous;
    vector_b_homogeneous << vector_b.x(), vector_b.y(), vector_b.z(), 1.0;

    // Transform the vector using the transformation matrix
    Vector4d vector_a_homogeneous = T_a_b * vector_b_homogeneous;

    // Convert back to Euclidean coordinates (3D vector)
    Vector3d vector_a(vector_a_homogeneous.x(), vector_a_homogeneous.y(), vector_a_homogeneous.z());

    return vector_a;
}

void extractTranslationAndRotation(const Matrix4d& transformationMatrix, Vector3d& translation, Vector3d& rotation)
{
    // Extract the translation vector (first three elements of the last column)
    translation = transformationMatrix.block<3, 1>(0, 3);

    // Extract the rotation vector (axis-angle representation)
    Eigen::Matrix3d rotationMatrix = transformationMatrix.block<3, 3>(0, 0);
    Eigen::AngleAxisd angleAxis(rotationMatrix);

    // The axis of rotation is stored in the angle-axis representation
    rotation = angleAxis.axis() * angleAxis.angle();
}