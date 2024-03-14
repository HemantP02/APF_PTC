
// Differential drive robot model
// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <functional>
// #include <numeric>

// // Define constants
// const double r = 15;  // Wheel radius
// const double s = 4 * r;  // Distance between wheel centers

// double interp(double x, const std::vector<double>& xValues, const std::vector<double>& yValues) {
//     size_t size = xValues.size();
//     if (size == 0 || size != yValues.size()) {
//         // Invalid input data, return NaN or throw an exception
//         return std::numeric_limits<double>::quiet_NaN();
//     }

//     size_t i = 0;
//     while (i < size && xValues[i] < x) {
//         ++i;
//     }

//     if (i == 0) {
//         return yValues[0];
//     } else if (i == size) {
//         return yValues[size - 1];
//     } else {
//         double x0 = xValues[i - 1];
//         double x1 = xValues[i];
//         double y0 = yValues[i - 1];
//         double y1 = yValues[i];
//         return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
//     }
// }

// // Function to calculate derivatives for odeint
// std::vector<double> diffModel(const std::vector<double>& x, double t, const std::vector<double>& dphiLA, const std::vector<double>& dphiRA, const std::vector<double>& timeVector) {
//     double dphiLt = interp(t, timeVector, dphiLA);
//     double dphiRt = interp(t, timeVector, dphiRA);

//     // Calculate derivatives
//     double dxdt = 0.5 * r * dphiLt * std::cos(x[2]) + 0.5 * r * dphiRt * std::cos(x[2]);
//     double dydt = 0.5 * r * dphiLt * std::sin(x[2]) + 0.5 * r * dphiRt * std::sin(x[2]);
//     double dthetadt = -r / s * dphiLt + r / s * dphiRt;

//     return { dxdt, dydt, dthetadt };
// }

// #include <vector>

// // Function to add two vectors element-wise
// std::vector<double> addVectors(const std::vector<double>& vec1, const std::vector<double>& vec2) {
//     std::vector<double> result;
//     for (size_t i = 0; i < vec1.size(); ++i) {
//         result.push_back(vec1[i] + vec2[i]);
//     }
//     return result;
// }

// // Function to multiply each element of a vector by a scalar
// std::vector<double> scalarMultiply(const std::vector<double>& vec, double scalar) {
//     std::vector<double> result;
//     for (size_t i = 0; i < vec.size(); ++i) {
//         result.push_back(vec[i] * scalar);
//     }
//     return result;
// }


// std::vector<double> rungeKutta4(const std::function<std::vector<double>(const std::vector<double>&, double, const std::vector<double>&, const std::vector<double>&, const std::vector<double>&)>& f, const std::vector<double>& initialState, const std::vector<double>& timePoints, const std::vector<double>& dphiLA, const std::vector<double>& dphiRA, const std::vector<double>& timeVector) {
//     std::vector<double> state = initialState;
//     std::vector<double> k1, k2, k3, k4;
//     double dt = timePoints[1] - timePoints[0];

//     std::vector<double> result;
//     for (double t : timePoints) {
//         result.push_back(state[0]);  // Store x
//         result.push_back(state[1]);  // Store y
//         result.push_back(state[2]);  // Store theta

//         k1 = f(state, t, dphiLA, dphiRA, timeVector);
//         k2 = f(addVectors(state, scalarMultiply(k1, 0.5 * dt)), t + 0.5 * dt, dphiLA, dphiRA, timeVector);
//         k3 = f(addVectors(state, scalarMultiply(k2, 0.5 * dt)), t + 0.5 * dt, dphiLA, dphiRA, timeVector);
//         k4 = f(addVectors(state, scalarMultiply(k3, dt)), t + dt, dphiLA, dphiRA, timeVector);

//         // Combine weighted averages of slopes to update state
//         for (size_t i = 0; i < state.size(); ++i) {
//             state[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * (dt / 6.0);
//         }
//     }
//     return result;
// }


// int main() {
//     // Define time vector
//     std::vector<double> timeVector(10000);
//     std::iota(std::begin(timeVector), std::end(timeVector), 0);  // 0 to 9999

//     // Define control variables: left wheel angular velocity and right wheel angular velocity
//     std::vector<double> dphiLA(timeVector.size(), 2);
//     std::vector<double> dphiRA(timeVector.size(), -1);

//     // Define initial state
//     std::vector<double> initialState = { 500, 500, 0 };  // x, y, theta

//     // Solve forward kinematics problem using Runge-Kutta 4th order integration
//     auto solution = rungeKutta4(diffModel, initialState, timeVector, dphiLA, dphiRA, timeVector);

//     // Save simulation data to a file
//     std::ofstream outputFile("simulationData.txt");
//     for (size_t i = 0; i < solution.size(); i += 3) {
//         outputFile << timeVector[i / 3] << " " << solution[i] << " " << solution[i + 1] << " " << solution[i + 2] << "\n";
//     }
//     outputFile.close();

//     // Output simulation result
//     std::cout << "Simulation data saved to 'simulationData.txt'.\n";

//     return 0;
// }

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <numeric>

// Function declarations
std::vector<double> addVectors(const std::vector<double>& vec1, const std::vector<double>& vec2);
std::vector<double> scalarMultiply(const std::vector<double>& vec, double scalar);
std::vector<double> diffModel(const std::vector<double>& x, double t, const std::vector<double>& v, double phi, const std::vector<double>& timeVector);
std::vector<double> rungeKutta4(const std::function<std::vector<double>(const std::vector<double>&, double, const std::vector<double>&, double, const std::vector<double>&)>& f, const std::vector<double>& initialState, const std::vector<double>& timePoints, const std::vector<double>& v, double phi, const std::vector<double>& timeVector);

// Define constants
const double L = 20;  // Distance between the wheels (wheelbase)
const double dt = 0.1; // Time step

// Function for linear interpolation
double interp(double x, const std::vector<double>& xValues, const std::vector<double>& yValues) {
    size_t size = xValues.size();
    if (size == 0 || size != yValues.size()) {
        // Invalid input data, return NaN or throw an exception
        return std::numeric_limits<double>::quiet_NaN();
    }

    size_t i = 0;
    while (i < size && xValues[i] < x) {
        ++i;
    }

    if (i == 0) {
        return yValues[0];
    } else if (i == size) {
        return yValues[size - 1];
    } else {
        double x0 = xValues[i - 1];
        double x1 = xValues[i];
        double y0 = yValues[i - 1];
        double y1 = yValues[i];
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }
}

// Function to calculate derivatives for odeint using bicycle model
std::vector<double> diffModel(const std::vector<double>& x, double t, const std::vector<double>& v, double phi, const std::vector<double>& timeVector) {
    double vx = v[0];
    double vy = v[1];
    double omega = v[2];

    // Extract current state variables
    double x_pos = x[0];
    double y_pos = x[1];
    double theta = x[2];

    // Calculate derivatives using bicycle model equations
    double dxdt = vx * std::cos(theta) - vy * std::sin(theta);
    double dydt = vx * std::sin(theta) + vy * std::cos(theta);
    double dthetadt = omega;

    return { dxdt, dydt, dthetadt };
}

// Runge-Kutta 4th order integration method
std::vector<double> rungeKutta4(const std::function<std::vector<double>(const std::vector<double>&, double, const std::vector<double>&, double, const std::vector<double>&)>& f, const std::vector<double>& initialState, const std::vector<double>& timePoints, const std::vector<double>& v, double phi, const std::vector<double>& timeVector) {
    std::vector<double> state = initialState;
    std::vector<double> k1, k2, k3, k4;

    std::vector<double> result;
    for (double t : timePoints) {
        result.push_back(state[0]);  // Store x
        result.push_back(state[1]);  // Store y
        result.push_back(state[2]);  // Store theta

        k1 = f(state, t, v, phi, timeVector);
        k2 = f(addVectors(state, scalarMultiply(k1, 0.5 * dt)), t + 0.5 * dt, v, phi, timeVector);
        k3 = f(addVectors(state, scalarMultiply(k2, 0.5 * dt)), t + 0.5 * dt, v, phi, timeVector);
        k4 = f(addVectors(state, scalarMultiply(k3, dt)), t + dt, v, phi, timeVector);

        // Combine weighted averages of slopes to update state
        for (size_t i = 0; i < state.size(); ++i) {
            state[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * (dt / 6.0);
        }
    }
    return result;
}

// Function to add two vectors element-wise
std::vector<double> addVectors(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    std::vector<double> result;
    for (size_t i = 0; i < vec1.size(); ++i) {
        result.push_back(vec1[i] + vec2[i]);
    }
    return result;
}

// Function to multiply each element of a vector by a scalar
std::vector<double> scalarMultiply(const std::vector<double>& vec, double scalar) {
    std::vector<double> result;
    for (size_t i = 0; i < vec.size(); ++i) {
        result.push_back(vec[i] * scalar);
    }
    return result;
}

int main() {
    // Define time vector
    std::vector<double> timeVector(10000);
    std::iota(std::begin(timeVector), std::end(timeVector), 0);  // 0 to 9999

    // Define control variables: forward velocity and steering angle
    std::vector<double> v(timeVector.size(), 5);    // Forward velocity
    double phi = 0.1;  // Steering angle (constant)

    // Define initial state
    std::vector<double> initialState = { 500, 500, 0 };  // x, y, theta

    // Solve forward kinematics problem using Runge-Kutta 4th order integration
    auto solution = rungeKutta4(diffModel, initialState, timeVector, v, phi, timeVector);

    // Save simulation data to a file
    std::ofstream outputFile("simulationData_bicycle.txt");
    for (size_t i = 0; i < solution.size(); i += 3) {
        outputFile << timeVector[i / 3] << " " << solution[i] << " " << solution[i + 1] << " " << solution[i + 2] << "\n";
    }
    outputFile.close();

    // Output simulation result
    std::cout << "Simulation data saved to 'simulationData_bicycle.txt'.\n";

    return 0;
}
