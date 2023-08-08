#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>

#ifndef UTILITY_H
#define UTILITY_H

using namespace std;
using namespace Eigen;
using namespace cv;

// Function to read CSV data into a 2D vector
vector<vector<float>> readCSV(const string& filename);

// Function to convert a 2D vector to cv::Mat
Mat vectorToMat(const vector<vector<float>>& data);

// Function to read camera matrix and distortion vector from CSV files and return as cv::Mat
void readCameraParameters(const string& cameraMatrixFile, const string& distortionVectorFile, Mat& camera_matrix, Mat& distortion_vector);

Matrix3d axisAngleToRotationMatrix(const Vector3d& axis_angle);

Matrix3d rollPitchYawToRotationMatrix(double roll, double pitch, double yaw);

Matrix4d buildTransformationMatrix(Matrix3d rotationMatrix, Vector3d translationVector);

Vector3d transformVector(const Matrix4d& T_a_b, const Vector3d& vector_b);

void extractTranslationAndRotation(const Matrix4d& transformationMatrix, Vector3d& translation, Vector3d& rotation);

#endif