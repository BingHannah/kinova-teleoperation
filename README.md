# Teleoperating-a-Kinova-Robot-Arm-Through-Real-time-Human-Arm-Motion-Imitation

## Overview

This repository contains the code and documentation for the research paper titled **"Teleoperating a Kinova Robot Arm Through Real-time Human Arm Motion Imitation."** This project develops two advanced algorithms for teleoperating a Kinova Gen-3 robotic arm by mimicking human arm movements in real-time. The algorithms are validated through both simulation and physical experiments, demonstrating their effectiveness in various teleoperation tasks.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Setup](#setup)
- [Usage](#usage)
  - [Simulation](#simulation)
  - [Physical Experiment](#physical-experiment)
- [Results](#results)
- [Acknowledgments](#acknowledgments)
- [License](#license)
- [Contact](#contact)

## Project Structure

The repository is organized as follows:

- **`src/`**: Contains the source code, including teleoperation algorithms, ROS nodes, and simulation scripts.
- **`docs/`**: Documentation files related to the project, including this README.
- **`figures/`**: Figures used in the research paper, including diagrams and experiment results.
- **`config/`**: Configuration files for ROS and other setup details.
- **`experiments/`**: Scripts and data for the experiments conducted with the Kinova Gen-3 robot arm.

## Installation

### Prerequisites

Ensure you have the following installed:

- **ROS (Robot Operating System)**: Tested with ROS Noetic on Ubuntu 20.04.
- **Kinova SDK**: For controlling the Kinova Gen-3 robotic arm.
- **Python 3.8+**: Required for running the scripts in this repository.

### Setup

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/kinova-teleoperation.git
   cd kinova-teleoperation
   
2. Install the required Python packages:
   
   ```bash
   pip install -r requirements.txt

3. Set up ROS workspace:
    
   ```bash
   source /opt/ros/noetic/setup.bash
   catkin_make
   source devel/setup.bash

4. Configure the Kinova Gen-3 robot and motion capture system as per the documentation in the config/ directory.

## Usage 

### Simulation

1. To run the teleoperation simulation using RViz:

   ```bash
   roslaunch kinova_teleoperation simulation.launch



### Physical Experiment
For physical experiments with the Kinova Gen-3 robot arm:

1. Ensure the robot is powered on and properly connected to your system.
2. Launch the teleoperation node:

   ```bash
   roslaunch kinova_teleoperation teleoperation.launch
   
3. Follow the instructions provided in the experiments/ directory to conduct specific experiments.

## Results
The experimental results demonstrate the effectiveness of the developed algorithms in mimicking human arm postures and achieving precise end-effector control. The results are documented in the results/ directory and include both simulation data and real-world experiment logs.

## Acknowledgments
This project was supported by [funding agency]. Special thanks to the team at [Institution Name] for their contributions and support throughout the project.

## License
This project is licensed under the MIT License. See the LICENSE file for more information.

## Contact
For any questions or collaboration inquiries, please contact:

Author1: [email@address.edu]
Author2: [email@address.edu]

You can also watch a video demonstration of our experiments on YouTube here:

   ```bash
   https://youtu.be/FRdjwJsbVTM?feature=shared


