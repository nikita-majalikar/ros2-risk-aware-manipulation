# Risk-Aware Manipulation (ROS 2)

## Overview

This project implements a **risk-aware manipulation pipeline in ROS 2**, focusing on **decision-making under uncertainty** rather than low-level robot control.

The system demonstrates how **perception uncertainty propagates through risk evaluation, planning policy selection, and execution monitoring**, following a modular ROS 2 architecture similar to real-world robotic systems.

All components communicate via ROS 2 topics and are intentionally lightweight and decoupled.

---

## High-Level Architecture

Each block is implemented as an **independent ROS 2 node** with a single responsibility.

---

## Node Descriptions

### 1️⃣ Perception Uncertainty Node

**Role:** Simulated perception system  
**Publishes:** `/object_pose_uncertain` (std_msgs/Float32)

This node simulates a vision pipeline by publishing a continuously changing uncertainty value between `0.0` and `1.0`, representing confidence in object pose estimation.

> *“How sure am I about where the object is?”*

---

### 2️⃣ Risk Evaluator Node

**Role:** Converts uncertainty into grasp risk  
**Subscribes:** `/object_pose_uncertain`  
**Publishes:** `/grasp_risk` (std_msgs/Float32)

This node transforms perception uncertainty into a grasp risk score.  
The risk value is bounded and represents the likelihood of a failed or unsafe grasp.

> *“Given perception uncertainty, how risky is this grasp?”*

---

### 3️⃣ Supervisor Node

**Role:** High-level decision-making and safety logic  
**Subscribes:**  
- `/grasp_risk`  
- `/execution_status`  

**Publishes:** `/planner_mode` (std_msgs/String)

The supervisor monitors both predicted risk and execution feedback, then selects an appropriate planning policy:

- `normal`
- `conservative`
- `abort`

This node acts as the **central decision authority** in the system.

> *“Should the robot proceed normally, slow down, or abort?”*

---

### 4️⃣ Planner Adapter (MoveIt Planner Node)

**Role:** Planning policy adapter (simulated MoveIt behavior)  
**Subscribes:** `/planner_mode`  
**Publishes:** `/planner_constraints` (std_msgs/String)

This node translates high-level supervisor decisions into planner behavior:

- `normal` → fast planning  
- `conservative` → slow/safe planning  
- `abort` → stop planning  

Actual MoveIt execution is simulated to focus on system architecture rather than configuration complexity.

> *“How aggressively should the robot plan?”*

---

### 5️⃣ Execution Monitor Node (Fake Execution)

**Role:** Simulated trajectory execution and runtime feedback  
**Publishes:**  
- `/fake_execution_progress` (std_msgs/Float32)  
- `/execution_status` (std_msgs/String)

This node simulates execution by publishing progress from `0.0 → 1.0` and execution states such as:

- `executing`
- `slow`
- `completed`

Execution feedback is sent back to the supervisor, closing the control loop.

> *“Is the robot executing, slowed down, or finished?”*

---

## Topic Interface Summary

| Topic | Message Type | Publisher | Subscriber |
|------|------------|-----------|------------|
| `/object_pose_uncertain` | Float32 | Perception Node | Risk Evaluator |
| `/grasp_risk` | Float32 | Risk Evaluator | Supervisor |
| `/planner_mode` | String | Supervisor | Planner Adapter |
| `/planner_constraints` | String | Planner Adapter | Planning Logic |
| `/execution_status` | String | Execution Monitor | Supervisor |
| `/fake_execution_progress` | Float32 | Execution Monitor | Debug / Visualization |

---

## Design Principles

- **Single Responsibility per Node**
- **Loose coupling via ROS 2 topics**
- **Feedback-driven decision making**
- **Safety-first planning logic**
- **Easily extensible to real robots and MoveIt**

---

## Scope

This project is designed to demonstrate:

- Risk-aware decision pipelines
- ROS 2 system architecture
- Runtime feedback integration
- Supervisor-based control logic
---

