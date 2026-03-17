# TurtleBot3 Autonomous Navigation with PlanSys2

## Overview

This project integrates **ROS2 PlanSys2** with a **TurtleBot3 Waffle** simulated in Gazebo to demonstrate autonomous task planning and execution. A PDDL-based planner generates a navigation plan, and a custom action performer executes it by driving the robot forward a fixed distance.

---

## System Architecture

The system is composed of three ROS2 packages:

### `tb3_planning`
Contains the PDDL domain and problem definitions, along with the planner solver configuration. The domain defines a `move` durative action that transitions a robot between locations. The problem instantiates the robot (`tb3`) and three waypoints (`wp1`, `wp2`, `wp3`), with the goal of moving the robot from `wp1` to `wp3`.

### `tb3_actions`
Implements the `move` action as a **PlanSys2 action performer** a lifecycle node that subscribes to the `/actions_hub` topic and responds to execution requests from the PlanSys2 executor. When activated, it drives the robot forward at a fixed velocity until a target distance is traveled, reporting progress back to the executor.

### `tb3_bringup`
Contains the launch file and a planner client node. The planner client populates the knowledge base with instances and predicates, requests a plan from POPF, and monitors execution until completion.

---

## Key Concepts

### PDDL Planning
The domain uses **STRIPS + typing + durative actions**. The `move` action has a duration of 10 seconds, removes the robot's current location predicate at the start, and asserts the new location at the end. POPF (Partial Order Planning Forward) solves the problem and returns a sequenced plan.

### PlanSys2 Executor and Action Hub
PlanSys2's executor dispatches actions by publishing **request messages** on the `/actions_hub` topic. Action performers subscribe to this topic and respond with a bid if their `action_name` parameter matches the requested action. The executor then activates the winning performer via the cascade lifecycle protocol.

### ActionExecutorClient Lifecycle Protocol
Action performers inherit from `plansys2::ActionExecutorClient`, which itself inherits from `rclcpp_cascade_lifecycle::CascadeLifecycleNode`. The correct startup sequence is:

1. The node is created and its `action_name` parameter is set to match the PDDL action name.
2. The node transitions to **Inactive** (Configured) state this sets up the `/actions_hub` subscription and the `/performers_status` publisher.
3. The PlanSys2 executor discovers the performer via the hub and triggers the **Active** transition automatically when it dispatches the action.
4. While active, `do_work()` is called periodically, sending feedback via `send_feedback()` and signaling completion via `finish()`.

A critical implementation detail: `trigger_transition(CONFIGURE)` must be called **after** the ROS executor begins spinning, because `CascadeLifecycleNode::on_configure()` creates publishers and subscriptions that require a running executor. This is achieved with a one-shot wall timer.

### The `action_name` Parameter
The executor matches performers to PDDL actions using the ROS parameter `action_name` on the performer node **not** the ROS node name and not the constructor argument used as the node name. Without this parameter explicitly set to `"move"`, `should_execute()` inside `ActionExecutorClient` silently rejects every dispatch request.

---

## Data Flow

```
PDDL Domain + Problem
        │
        ▼
   POPF Planner  ──▶  Plan: (move tb3 wp1 wp3)
        │
        ▼
  PlanSys2 Executor
        │  publishes ActionExecution{type=REQUEST, action="move"}
        ▼
   /actions_hub topic
        │  received by /move node (action_name="move")
        ▼
  MoveAction::do_work()
        │  publishes /cmd_vel -> TurtleBot3 drives forward
        │  sends feedback (0% -> 100%) back to executor
        ▼
  finish(true) -> executor marks plan as SUCCEEDED
```

---

## What the Robot Does

The TurtleBot3 drives **straight forward at 0.2 m/s** until it has traveled 1.5 meters, as measured by odometry. Waypoint coordinates are abstract in this implementation the planner reasons about symbolic locations, while the action performer handles only the physical motion primitive.