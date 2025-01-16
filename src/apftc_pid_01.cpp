#include <iostream>
#include <vector>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Define a structure for a point in 2D space
struct Point {
    double x;
    double y;

    // Overload addition operator for Point struct
    Point operator+(const Point& other) const {
        return {x + other.x, y + other.y};
    }
};

// Function to calculate the distance between two points
double distance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}



// Function to calculate the attractive force
Point attractiveForce(const Point& current, const Point& goal, double k_att) {
    double distance_to_goal = distance(current, goal);
    return {(k_att * (goal.x - current.x)) / distance_to_goal, (k_att * (goal.y - current.y)) / distance_to_goal};
}

Point dynamicRepulsiveForce(const Point& current, const std::vector<std::pair<Point, Point>>& obstacles_with_velocities,
                            double k_rep, double d_0, double vehicle_velocity, double& front_dynamic_radius, double& rear_dynamic_radius,
                            double k_v,double angle_to_obstacle) {
    Point total_force = {0, 0};

    // Initialize variables for closest obstacles and relative velocities
    double closest_front_distance = std::numeric_limits<double>::max();
    double closest_rear_distance = std::numeric_limits<double>::max();
    double front_obstacle_radius = 0;
    double rear_obstacle_radius = 0;
    double relative_front_velocity = 0;
    double relative_rear_velocity = 0;

    // Predict future position of the robot based on its velocity
    Point future_robot_position = current;
    future_robot_position.x += vehicle_velocity; // Assuming movement along the x-axis only

    // Determine closest obstacles within dynamic regions and predict their future positions
    for (const auto& obstacle_with_velocity : obstacles_with_velocities) {
        const Point& obstacle = obstacle_with_velocity.first;
        const Point& obstacle_velocity = obstacle_with_velocity.second;

        // Predict future position of the obstacle based on its velocity
        Point future_obstacle_position = obstacle;
        future_obstacle_position.x += obstacle_velocity.x; // Assuming movement along the x-axis only

        // Calculate relative velocity between obstacle and robot
        double relative_velocity_x = obstacle_velocity.x - vehicle_velocity; // Assuming movement along the x-axis only

        double angle_to_obstacle = std::atan2(obstacle.y - current.y, obstacle.x - current.x);
        double distance_to_obstacle = distance(current, obstacle);
        double obstacle_radius = 0.5; // You need to define this based on your obstacle definition

        // Front detection zone (120 degrees)
        if (angle_to_obstacle >= -M_PI / 3 && angle_to_obstacle <= M_PI / 3) {
            if (distance_to_obstacle < closest_front_distance) {
                closest_front_distance = distance_to_obstacle;
                front_obstacle_radius = obstacle_radius;
                relative_front_velocity = relative_velocity_x; // Use relative velocity along x-axis
            }
        }

        // Rear detection zone (240 degrees)
        if (angle_to_obstacle >= -2 * M_PI / 3 && angle_to_obstacle <= 2 * M_PI / 3) {
            if (distance_to_obstacle < closest_rear_distance) {
                closest_rear_distance = distance_to_obstacle;
                rear_obstacle_radius = obstacle_radius;
                relative_rear_velocity = relative_velocity_x; // Use relative velocity along x-axis
            }
        }
    }

    // Calculate dynamic radii for repulsion
    if (relative_front_velocity > 10) {
        front_dynamic_radius = closest_front_distance + 10 * front_obstacle_radius;
    } else if (relative_front_velocity > 4) {
        front_dynamic_radius = closest_front_distance + relative_front_velocity * front_obstacle_radius;
    } else {
        front_dynamic_radius = closest_front_distance + 4 * front_obstacle_radius;
    }

    if (relative_rear_velocity < -6) {
        rear_dynamic_radius = closest_rear_distance + 6 * rear_obstacle_radius;
    } else if (relative_rear_velocity <= 2) {
        rear_dynamic_radius = closest_rear_distance + std::abs(relative_rear_velocity) * rear_obstacle_radius;
    } else {
        rear_dynamic_radius = closest_rear_distance + 2 * rear_obstacle_radius;
    }

    // Calculate repulsive force within dynamic regions using future points
    for (const auto& obstacle_with_velocity : obstacles_with_velocities) {
        const Point& obstacle = obstacle_with_velocity.first;
        const Point& obstacle_velocity = obstacle_with_velocity.second;

        // Predict future position of the obstacle based on its velocity
        Point future_obstacle_position = obstacle;
        future_obstacle_position.x += obstacle_velocity.x; // Assuming movement along the x-axis only

        double distance_to_obstacle = distance(current, future_obstacle_position);
        double obstacle_radius = 0.5; // You need to define this based on your obstacle definition

        // Check if obstacle is within dynamic front or rear region
        if (distance_to_obstacle <= front_dynamic_radius || distance_to_obstacle <= rear_dynamic_radius) {
            // Calculate repulsive strength
            double repulsive_strength = k_rep * (1 / distance_to_obstacle - 1 / d_0) * (1 / std::pow(distance_to_obstacle, 2));

            // Calculate repulsive force direction
            double repulsive_force_x = repulsive_strength * (current.x - future_obstacle_position.x) / distance_to_obstacle;
            double repulsive_force_y = repulsive_strength * (current.y - future_obstacle_position.y) / distance_to_obstacle;

            // Add repulsive force to total force
            total_force.x += repulsive_force_x;
            total_force.y += repulsive_force_y;
        }
    }

    // Velocity repulsive force calculation
    if (relative_front_velocity > 0 && angle_to_obstacle >= -M_PI / 3 && angle_to_obstacle <= M_PI / 3) {
        total_force.x += k_v * relative_front_velocity; // Front obstacle velocity repulsion
    }

    if (relative_rear_velocity < 0 && angle_to_obstacle >= -2 * M_PI / 3 && angle_to_obstacle <= 2 * M_PI / 3) {
        total_force.x += k_v * relative_rear_velocity; // Rear obstacle velocity repulsion
    }

    return total_force;
}



// Function to calculate the road repulsive force
double roadRepulsiveForce(double x, double x_l, double x_r, double L, double k_road1, double k_road2, double k_road3) {
    if (x <= L / 4) {
        return k_road1 * (exp((x - x_l) / L) - 1);
    } else if (x < 3 * L / 4) {
        return 2 * k_road2 * cos(2 * (x - x_l) / L);
    } else {
        return k_road3 * (exp((x - x_r) / L) - 1);
    }
}

// Function to perform APF path planning
std::vector<Point> apfPathPlanning(const Point& start, const Point& goal, const std::vector<std::pair<Point, Point>>& obstacles_with_velocities,
                                   double k_att, double k_rep, double d_0, double step_size, int max_iterations, double vehicle_velocity,
                                   double k_road1, double k_road2, double k_road3, double L, double x_l, double x_r, double k_v) {
    std::vector<Point> path;
    path.push_back(start);
    Point current = start;

    double front_dynamic_radius = 0;
    double rear_dynamic_radius = 0;

    for (int i = 0; i < max_iterations; ++i) {
        for (const auto& obstacle_with_velocity : obstacles_with_velocities) {
            const Point& obstacle = obstacle_with_velocity.first;
            double angle_to_obstacle = std::atan2(obstacle.y - current.y, obstacle.x - current.x);
            Point force = attractiveForce(current, goal, k_att) +
                        dynamicRepulsiveForce(current, obstacles_with_velocities, k_rep, d_0, vehicle_velocity, front_dynamic_radius, rear_dynamic_radius, k_v, angle_to_obstacle);

            // Calculate lateral position of the robot relative to the road boundaries
            double lateral_position = current.y;

            // Calculate road repulsive force
            double road_force = roadRepulsiveForce(lateral_position, x_l, x_r, L, k_road1, k_road2, k_road3);

            // Add road repulsive force to the total force
            force.y += road_force;

            double force_magnitude = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2));
            if (force_magnitude < 0.001) {
                // If force is very small, consider it as convergence
                break;
            }
            // Normalize the force vector
            double norm_x = force.x / force_magnitude;
            double norm_y = force.y / force_magnitude;
            // Update the current position
            current.x += step_size * norm_x;
            current.y += step_size * norm_y;
            path.push_back(current);
            // Check if reached the goal
            if (distance(current, goal) < step_size) {
                path.push_back(goal);
                break;
            }
        }
    }


    return path;
}

class PIDController {
public:
    // Constructor
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0) {}

    // Compute control output
    double compute(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        double output = kp * error + ki * integral + kd * derivative;
        prev_error = error;
        return output;
    }

private:
    double kp, ki, kd;
    double prev_error;
    double integral;
};

int main() {
    // Define the start and goal points
    Point start = {-4, 0};
    Point goal = {30, 0};

    // Define the obstacles with velocities and initial positions
    std::vector<std::pair<Point, Point>> obstacles_with_velocities = {
        {{9, 0}, {-0.1, 0}},    // obstacle at (9, 9) with velocity (1, 0)
        {{7, -0.1}, {0.1, 0}},    // obstacle at (7, 6) with velocity (0, 0)
        {{5, -0.5}, {0.1, 0}},
        {{-1,-0.1}, {0.1, 0}},
        {{-1, -0.2}, {0.1, 0}}
    };

    // Parameters
    double k_att = 3.0;
    double k_rep = 1.0;
    double k_v = 1.0; // Velocity gain coefficient for velocity repulsive force
    double d_0 = 1.0;
    double step_size = 0.1;
    int max_iterations = 1000;
    double vehicle_velocity = 1.0; // Fake vehicle velocity

    // Road parameters
    double k_road1 = 1.0;
    double k_road2 = 1.0;
    double k_road3 = 1.0;
    double L = 2.0; // Width of the road
    double x_l = -L / 2; // Horizontal position of the centerline of the left lane
    double x_r = L / 2; // Horizontal position of the centerline of the right lane

        // Define PID controller parameters
    const double kp_v = 5.1;
    const double ki_v = 0.01;
    const double kd_v = 0.51;

    const double kp_phi = 1.1;
    const double ki_phi = 0.01;
    const double kd_phi = 0.51;

    double theta = 0;   // Initial orientation (assumed to be 0 for simplicity)
    double v = 0;       // Initial linear velocity
    double phi = 0;     // Initial steering angle
    double dt = 0.1;

    // Initialize variables for PID controller
    double prev_error = 0;
    double integral = 0;

    // Perform APF path planning
    std::vector<Point> path = apfPathPlanning(start, goal, obstacles_with_velocities, k_att, k_rep, d_0, step_size, max_iterations, vehicle_velocity,
                                              k_road1, k_road2, k_road3, L, x_l, x_r, k_v);

    std::vector<double> path_x, path_y;
    for (const auto& point : path) {
        path_x.push_back(point.x);
        path_y.push_back(point.y);
    }
    // Plot the trajectory planned by the APF algorithm
    // plt::plot(path_x, path_y, "g--");
    std::vector<double> robot_trajectory_x, robot_trajectory_y;

    // Initialize PID controller for controlling linear velocity
    PIDController velocity_controller(kp_v, ki_v, kd_v);

    // Initialize PID controller for controlling steering angle
    PIDController steering_controller(kp_phi, ki_phi, kd_phi);
    // Initialize robot position to start
    double x = start.x; // Initial x position
    double y = start.y; // Initial y position

    // Iterate through the trajectory and apply control inputs
    for (size_t i = 0; i < path.size() - 1; ++i) {
        // Calculate error between current position and desired position on the trajectory
        double error_x = path[i+1].x - x;
        double error_y = path[i+1].y - y;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

        // Compute control output using PID controller
        double control_output = velocity_controller.compute(distance_error, dt);

        // Apply control output to adjust linear velocity
        v = control_output;

        // Calculate desired orientation (angle to next point on the trajectory)
        double desired_theta = std::atan2(error_y, error_x);

        // Compute error in orientation
        double angle_error = desired_theta - theta;

        // Ensure angle error is within range [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // Compute control output for steering angle using PID controller
        control_output = steering_controller.compute(angle_error, dt);

        // Apply control output to adjust steering angle
        phi = control_output;

        // Update state of the robot using bicycle model kinematics
        // Compute next state using bicycle model kinematics
        double x_prime = x + v * std::cos(theta) * dt;
        double y_prime = y + v * std::sin(theta) * dt;
        double theta_prime = theta + (v / L) * std::tan(phi) * dt;

        // Update current state with next state
        x = x_prime;
        y = y_prime;
        theta = theta_prime;

        // Store the robot's trajectory
        robot_trajectory_x.push_back(x);
        robot_trajectory_y.push_back(y);

        }
    // Plot the robot's trajectory
    plt::plot(robot_trajectory_x, robot_trajectory_y, "r-");

    // Add labels and legend
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("APF Path Planning with Bicycle Model");
    plt::legend();

    // Show plot
    plt::show();

    return 0;
}