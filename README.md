# ROS2 Risk-Aware Manipulation System

## Overview
Multi-node ROS2 system implementing risk-aware decision-making for robotic manipulation.
Designed to explore system-level ROS2 behavior, inter-node communication, and runtime debugging.

## Architecture
Nodes:
- Risk evaluator node
- Planner adapter node
- Execution supervisor node

Communication via ROS2 topics and parameters.

## Tech Stack
ROS2 Humble, Python, MoveIt, Gazebo, RViz

## How to Run
colcon build
source install/setup.bash
ros2 launch risk_aware_manipulation <launch_file>.launch.py

## What This Project Demonstrates
- ROS2 multi-node architecture
- Topic-based debugging and parameter tuning
- Simulation-based validation using Gazebo and RViz
