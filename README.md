# APF_PTC

This repository contains an implementation of Artificial Potential Field (APF) for path planning and a PID controller for target tracking in robotics. The project aims to demonstrate obstacle avoidance and precise movement control for autonomous navigation.

## Features

- **Artificial Potential Field Algorithm**: Used for path planning to guide robots towards a target while avoiding obstacles.
- **PID Controller**: Ensures accurate path tracking and stability.
- **Multilingual Implementation**: C++ and Python scripts for flexibility.

## File Structure

- `apft_05.cpp` and similar: C++ implementations of the APF path planning.
- `apftc_c.py`: Python script for controller-based navigation.
- `apftc_pid_01.cpp`: Combines path planning with PID control.
- `test1.py`: Example for testing purposes.

## Dependencies

- Standard C++11 or higher compiler
- Python 3.x
- Libraries: `matplotlib` (for Python visualization)

## How to Use

1. Compile the C++ files using:
   ```
   g++ apftc_pid_01.cpp -o apf_pid -std=c++11
   ./apf_pid
   ```
2. Run the Python script:
   ```
   python3 apftc_c.py
   ```

## Future Enhancements

- Integration with ROS for real-world robotics applications.
- Dynamic obstacle handling.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your improvements.

## License

This project is licensed under the MIT License.

## Contact

For inquiries, reach out to the repository owner on [GitHub](https://github.com/HemantP02/APF_PTC).
