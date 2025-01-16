import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, pow, atan2, exp, cos, sin, tan

# Define a class for a point in 2D space
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Overload addition operator for Point class
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

# Function to calculate the distance between two points
def distance(p1, p2):
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2))

# Function to calculate the attractive force
def attractive_force(current, goal, k_att):
    distance_to_goal = distance(current, goal)
    return Point((k_att * (goal.x - current.x)) / distance_to_goal, (k_att * (goal.y - current.y)) / distance_to_goal)

# Function to calculate the dynamic repulsive force
def dynamic_repulsive_force(current, obstacles_with_velocities, k_rep, d_0, vehicle_velocity, front_dynamic_radius, rear_dynamic_radius, k_v, angle_to_obstacle):
    total_force = Point(0, 0)

    # Initialize variables for closest obstacles and relative velocities
    closest_front_distance = float('inf')
    closest_rear_distance = float('inf')
    front_obstacle_radius = 0
    rear_obstacle_radius = 0
    relative_front_velocity = 0
    relative_rear_velocity = 0

    # Predict future position of the robot based on its velocity
    future_robot_position = Point(current.x + vehicle_velocity, current.y)

    # Determine closest obstacles within dynamic regions and predict their future positions
    for obstacle_with_velocity in obstacles_with_velocities:
        obstacle, obstacle_velocity = obstacle_with_velocity

        # Predict future position of the obstacle based on its velocity
        future_obstacle_position = Point(obstacle.x + obstacle_velocity.x, obstacle.y)

        # Calculate relative velocity between obstacle and robot
        relative_velocity_x = obstacle_velocity.x - vehicle_velocity

        angle_to_obstacle = atan2(obstacle.y - current.y, obstacle.x - current.x)
        distance_to_obstacle = distance(current, obstacle)
        obstacle_radius = 0.5 # You need to define this based on your obstacle definition

        # Front detection zone (120 degrees)
        if -np.pi / 3 <= angle_to_obstacle <= np.pi / 3:
            if distance_to_obstacle < closest_front_distance:
                closest_front_distance = distance_to_obstacle
                front_obstacle_radius = obstacle_radius
                relative_front_velocity = relative_velocity_x # Use relative velocity along x-axis

        # Rear detection zone (240 degrees)
        if -2 * np.pi / 3 <= angle_to_obstacle <= 2 * np.pi / 3:
            if distance_to_obstacle < closest_rear_distance:
                closest_rear_distance = distance_to_obstacle
                rear_obstacle_radius = obstacle_radius
                relative_rear_velocity = relative_velocity_x # Use relative velocity along x-axis

    # Calculate dynamic radii for repulsion
    if relative_front_velocity > 10:
        front_dynamic_radius = closest_front_distance + 10 * front_obstacle_radius
    elif relative_front_velocity > 4:
        front_dynamic_radius = closest_front_distance + relative_front_velocity * front_obstacle_radius
    else:
        front_dynamic_radius = closest_front_distance + 4 * front_obstacle_radius

    if relative_rear_velocity < -6:
        rear_dynamic_radius = closest_rear_distance + 6 * rear_obstacle_radius
    elif relative_rear_velocity <= 2:
        rear_dynamic_radius = closest_rear_distance + abs(relative_rear_velocity) * rear_obstacle_radius
    else:
        rear_dynamic_radius = closest_rear_distance + 2 * rear_obstacle_radius

    # Calculate repulsive force within dynamic regions using future points
    for obstacle_with_velocity in obstacles_with_velocities:
        obstacle, obstacle_velocity = obstacle_with_velocity

        # Predict future position of the obstacle based on its velocity
        future_obstacle_position = Point(obstacle.x + obstacle_velocity.x, obstacle.y)

        distance_to_obstacle = distance(current, future_obstacle_position)
        obstacle_radius = 0.5 # You need to define this based on your obstacle definition

        # Check if obstacle is within dynamic front or rear region
        if distance_to_obstacle <= front_dynamic_radius or distance_to_obstacle <= rear_dynamic_radius:
            # Calculate repulsive strength
            repulsive_strength = k_rep * (1 / distance_to_obstacle - 1 / d_0) * (1 / pow(distance_to_obstacle, 2))

            # Calculate repulsive force direction
            repulsive_force_x = repulsive_strength * (current.x - future_obstacle_position.x) / distance_to_obstacle
            repulsive_force_y = repulsive_strength * (current.y - future_obstacle_position.y) / distance_to_obstacle

            # Add repulsive force to total force
            total_force.x += repulsive_force_x
            total_force.y += repulsive_force_y

    # Velocity repulsive force calculation
    if relative_front_velocity > 0 and -np.pi / 3 <= angle_to_obstacle <= np.pi / 3:
        total_force.x += k_v * relative_front_velocity # Front obstacle velocity repulsion

    if relative_rear_velocity < 0 and -2 * np.pi / 3 <= angle_to_obstacle <= 2 * np.pi / 3:
        total_force.x += k_v * relative_rear_velocity # Rear obstacle velocity repulsion

    return total_force

# Function to calculate the road repulsive force
def road_repulsive_force(x, x_l, x_r, L, k_road1, k_road2, k_road3):
    if x <= L / 4:
        return k_road1 * (exp((x - x_l) / L) - 1)
    elif x < 3 * L / 4:
        return 2 * k_road2 * cos(2 * (x - x_l) / L)
    else:
        return k_road3 * (exp((x - x_r) / L) - 1)

# Function to perform APF path planning
def apf_path_planning(start, goal, obstacles_with_velocities, k_att, k_rep, d_0, step_size, max_iterations, vehicle_velocity, k_road1, k_road2, k_road3, L, x_l, x_r, k_v):
    path = [start]
    current = start

    front_dynamic_radius = 0
    rear_dynamic_radius = 0

    for _ in range(max_iterations):
        for obstacle_with_velocity in obstacles_with_velocities:
            obstacle, obstacle_velocity = obstacle_with_velocity
            
            angle_to_obstacle = atan2(obstacle.y - current.y, obstacle.x - current.x)
            force = attractive_force(current, goal, k_att) + \
                    dynamic_repulsive_force(current, obstacles_with_velocities, k_rep, d_0, vehicle_velocity, front_dynamic_radius, rear_dynamic_radius, k_v, angle_to_obstacle)

            # Calculate lateral position of the robot relative to the road boundaries
            lateral_position = current.y

            # Calculate road repulsive force
            road_force = road_repulsive_force(lateral_position, x_l, x_r, L, k_road1, k_road2, k_road3)

            # Add road repulsive force to the total force
            force.y += road_force

            force_magnitude = sqrt(pow(force.x, 2) + pow(force.y, 2))
            if force_magnitude < 0.001:
                # If force is very small, consider it as convergence
                break
            # Normalize the force vector
            norm_x = force.x / force_magnitude
            norm_y = force.y / force_magnitude
            # Update the current position
            current.x += step_size * norm_x
            current.y += step_size * norm_y
            path.append(Point(current.x, current.y))
            # Check if reached the goal
            if distance(current, goal) < step_size:
                path.append(goal)
                break

    return path


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def bicycle_model(x, y, theta, v, phi, L, dt):
    theta = np.arctan2(np.sin(theta), np.cos(theta))  # Normalize theta to the range [-pi, pi]
    x_prime = x + v * np.cos(theta) * dt
    y_prime = y + v * np.sin(theta) * dt
    theta_prime = theta + (v / L) * np.tan(phi) * dt
    return x_prime, y_prime, theta_prime

def main():


    # Define the start and goal points
    start = Point(-4, 0)
    goal = Point(40, 0)

    # Define the obstacles with velocities and initial positions
    obstacles_with_velocities = [
        (Point(9, 0), Point(-0.1, 0)),    # obstacle at (9, 9) with velocity (1, 0)
        (Point(7, -0.1), Point(0.5, 0)),    # obstacle at (7, 6) with velocity (0, 0)
        (Point(5, -0.5), Point(0.1, 0)),
        (Point(-1,-0.1), Point(0.5, 0)),
        (Point(-1, -0.2), Point(0.1, 0))
    ]

    # Parameters
    k_att = 3.0
    k_rep = 1.0
    k_v = 1.0  # Velocity gain coefficient for velocity repulsive force
    d_0 = 1.0
    step_size = 0.1
    max_iterations = 200
    vehicle_velocity = 1.0  # Fake vehicle velocity

    # Road parameters
    k_road1 = 1.0
    k_road2 = 1.0
    k_road3 = 1.0
    L = 2.0  # Width of the road
    x_l = -L / 2  # Horizontal position of the centerline of the left lane
    x_r = L / 2  # Horizontal position of the centerline of the right lane

    x = start.x
    y = start.y

    # Perform APF path planning
    path = apf_path_planning(start, goal, obstacles_with_velocities, k_att, k_rep, d_0, step_size, max_iterations, vehicle_velocity,
    
                            k_road1, k_road2, k_road3, L, x_l, x_r, k_v)
    path.pop(0)
    path.insert(0, Point(x, y))
    
    path_x = [point.x for point in path]
    path_y = [point.y for point in path]
    plt.plot(path_x, path_y, "go",label = "APF_Trajectory")

    # Define PID controller parameters
    kp_v = 4.05050
    ki_v = 0.0001
    kd_v = 0.51

    kp_phi = 4.05051
    ki_phi = 0.0001
    kd_phi = 0.51
    threshold = 0.3

        # Initialize robot state

    v = 0      # Initial linear velocity
    phi = 0    # Initial steering angle
    dt = 0.1   # Time step

    # Update robot's state to match the starting point of the APF path
    theta = np.arctan2(path[1].y - path[0].y, path[1].x - path[0].x)

    # Initialize PID controllers
    velocity_controller = PIDController(kp_v, ki_v, kd_v)
    steering_controller = PIDController(kp_phi, ki_phi, kd_phi)


    # Robot trajectory storage
    robot_trajectory_x = [x]
    robot_trajectory_y = [y]

    # Iterate through the trajectory and apply control inputs
    for i in range(len(path) - 1):
        error_x = path[i + 1].x - x
        error_y = path[i + 1].y - y
        distance_error = sqrt(error_x * error_x + error_y * error_y)
        print(distance_error)

        # Compute control output for linear velocity
        control_output_v = velocity_controller.compute(distance_error, dt)
        v = control_output_v
        # If the distance to the next point is smaller than a threshold, stop

        # Compute desired orientation
        desired_theta = atan2(error_y, error_x)
        angle_error = desired_theta - theta

        # Ensure angle error is within range [-pi, pi]
        if angle_error > np.pi:
            angle_error -= 2 * np.pi
        if angle_error < -np.pi:
            angle_error += 2 * np.pi

        # Compute control output for steering angle
        control_output_phi = steering_controller.compute(angle_error, dt)
        phi = control_output_phi
        if distance_error > threshold:
            v = 0
        else:
            v = control_output_v
        # Update robot state using bicycle model kinematics
        x, y, theta = bicycle_model(x, y, theta, v, phi, L, dt)


        # Store robot trajectory
        robot_trajectory_x.append(x)
        robot_trajectory_y.append(y)
        if distance_error > threshold:
            break

    # Plot the robot's trajectory
    plt.plot(robot_trajectory_x, robot_trajectory_y, "r-",label = "Robot_PID_Trajectory")

    # Show plot
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("APF Path Planning with Bicycle Model")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()



