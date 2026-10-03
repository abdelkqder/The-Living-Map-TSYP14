# 🗺️ THE LIVING MAP

### Spatial Memory for Emergency Robots

**TSYP14 Technical Challenge — The Living Map**
**IEEE RAS × IEEE AESS Tunisia Section**
**IEEE ENIB Student Branch — Tunisia**

> **Give the environment a memory that survives the robot.**

---

## 🚨 The Challenge

Emergency robots operating inside **GPS-denied environments such as mines and tunnels** face a fundamental problem:

A robot may discover critical information — a fire, gas infiltration, victim, blocked passage — and then leave the area, lose communication, or fail.

When the next robot enters, should it have to start from zero?

**The Living Map explores a different approach: give the environment its own persistent spatial memory.**

---

## 💡 Our Approach

We are developing an **Adaptive Mission-Aware Spatial Memory** system in which information discovered by one robot can remain useful after that robot is gone.

Instead of keeping knowledge only inside a robot, the system preserves selected information in **distributed radio beacons** placed throughout the environment.

The information can then be:

**discovered → preserved → communicated → verified → updated → reused**

As the environment changes, the map changes with it.

---

## 🧠 System Architecture

```text
                 GPS-DENIED ENVIRONMENT
              ┌───────────────────────────┐
              │                           │
              │      🤖 WRITER ROBOT      │
              │      Explore & Sense      │
              │            │              │
              │            ▼              │
              │      📡 BEACON NETWORK    │
              │      Spatial Memory       │
              │            │              │
              │      Multi-hop Relay      │
              │            │              │
              └────────────┼──────────────┘
                           ▼
                    📡 ONA GATEWAY
                  Outside Network Area
                           │
                           │ Wi-Fi
                           ▼
                    🖥️ COMMAND POST
                  Living Map & Planning
                           │
                           │ Mission Brief
                           ▼
                 🤖 EXECUTOR FLEET
             Fire • Gas • Victim • Debris
                           │
                           ▼
                  Memory Verification
                       & Update
                           │
                           └──────► Living Map
```

### Architectural Constraint

There is **no direct Robot ↔ Command Post communication**.

Information from the disconnected environment must pass through the **ONA and beacon communication network**.

---

## 🔄 The Living Map Loop

The system is built around a continuous information cycle:

```text
SENSE
  ↓
LOCALIZE
  ↓
UNDERSTAND
  ↓
PRESERVE CRITICAL INFORMATION
  ↓
STORE IN SPATIAL MEMORY
  ↓
COMMUNICATE THROUGH BEACONS
  ↓
COMMAND POST UPDATES THE LIVING MAP
  ↓
PRIORITIZE & ASSIGN A MISSION
  ↓
EXECUTOR USES INHERITED MEMORY
  ↓
VERIFY / DISCOVER CHANGES
  ↓
UPDATE THE MEMORY
  ↓
NEXT MISSION
```

The objective is not simply to record what a robot saw.

It is to preserve **what may still be useful for the next mission**.

---

## ✨ Key Concepts

### 🧭 Autonomous Exploration

The Writer explores a partially unknown environment using frontier-based exploration rather than following a fixed route.

It builds its own discovered map, handles obstacles and dead ends, and can return to the entry point.

### 📡 Spatial Memory & Multi-hop Communication

Beacons act as persistent memory nodes and communication relays.

Information from deeper parts of the tunnel can travel through neighboring beacons toward the ONA using a simulated multi-hop network.

### 🌍 Local → Global Coordinates

The Writer maintains a local pose `(x, y, θ)`.

The ONA transforms local coordinates into a global geographic reference used by the Command Post.

### 🧠 Adaptive Memory

Memory records contain information such as:

* event type
* local and global coordinates
* severity
* confidence
* timestamps and age
* source
* version
* verification status
* passage accessibility

Information can evolve from:

`UNVERIFIED → VERIFIED → ACTIVE → CLEARED`

or become:

`CONTRADICTED / ESCALATED`

### 🔀 Dynamic Environment

The environment is not assumed to remain static.

For example, debris may appear **after the Writer has already left**.

An Executor can discover the change, re-plan around it, preserve the new information, and update the Living Map.

### 🤖 Mission-Aware Executor Fleet

The Command Post matches missions with available executor capabilities:

| Executor  | Main mission                    |
| --------- | ------------------------------- |
| 🔥 FIRE   | Fire / thermal response         |
| 🧍 VICTIM | Victim investigation / response |
| ☁️ GAS    | Gas infiltration investigation  |
| 🧱 DEBRIS | Blockage / debris removal       |

Executors report their mission results and return to an operational state for future missions, or report failure/service requirements.

---

## 🎬 Phase 1 Simulations

The repository contains several complementary simulations rather than relying on one scenario.

### 1. Global Living Map

Demonstrates the complete system:

**Writer → Beacons → ONA → Command Post → Executor → Memory Update**

### 2. Writer Exploration

Focuses on:

* autonomous exploration
* map coverage
* dead ends
* obstacles
* alternative paths
* event discovery
* return to entry

### 3. Executor Fleet

Starts from an existing Living Map and demonstrates:

* priority analysis
* executor selection
* mission assignment
* navigation
* task execution
* memory verification
* return
* fleet availability

### 4. Dynamic Debris

Demonstrates a changing environment:

```text
Writer leaves
      ↓
New debris appears
      ↓
Executor encounters unexpected blockage
      ↓
Re-planning
      ↓
Blockage preserved in memory
      ↓
Debris Executor dispatched
      ↓
Obstacle cleared
      ↓
Original mission continues
```

### 5. Living Map Comparison

The project also includes a simulation comparison between:

**Executor without inherited memory**

and

**Executor using the Living Map**

using the same simulated environment to measure mission behavior.

---

## 🧪 Phase 1 Software Proof of Concept

This repository provides the **software and simulation foundation** for the project.

### Implemented / Simulated

* Autonomous Writer exploration
* Dynamic obstacle handling
* Persistent spatial memory
* Blockage memory
* Multi-hop beacon communication
* Packet validation and retry
* ONA gateway
* Local-to-global coordinate transformation
* Living Map
* Memory ageing and versioning
* Mission prioritization
* Capability-based executor assignment
* Specialized Executor state machines
* Executor feedback and memory updates
* Dynamic debris scenarios
* Writer failure / memory continuity
* Headless simulations
* Automated test suite

### Still Planned for the Physical Prototype

* ESP32 robot hardware
* physical LoRa communication
* physical beacon deployment
* real sensors and actuators
* real-world localization validation
* battery and sensor characterization

The simulations model these future physical interfaces but **do not claim physical measurements**.

---

## 🚀 Quick Start

### Install

```bash
python -m venv .venv
```

Activate the environment and install dependencies:

```bash
pip install -r requirements.txt
```

### Run tests

```bash
python -m pytest -q
```

### Run the demonstrations

```bash
python -m simulation.global_demo --seed 42

python -m simulation.writer_demo --seed 42

python -m simulation.fleet_demo --seed 42

python -m simulation.dynamic_debris --seed 42

python -m simulation.compare_memory --seeds 6
```

Interactive Pygame simulation:

```bash
python run_simulation.py --seed 42
```

---

## 📊 Repository Structure

```text
common/           Protocols, memory, coordinates, poses, missions
communication/    Simulated mesh / transport interfaces
beacon/           Memory nodes, relay, deployment
gateway/          ONA gateway
writer_robot/     Exploration, perception, memory decisions
executor_robot/   Executor capabilities and state machine
command_post/     Living Map, planning, fleet management
navigation/       A* and frontier exploration
simulation/       World, scenarios, demos and visualization
tests/            Automated tests
docs/             Architecture, protocol, failure analysis
```

---

## 📚 Documentation

📘 [Phase 1 Demonstration Guide](docs/phase1_demo.md)

📋 [Phase 1 Requirement Traceability](docs/phase1_traceability.md)

🏗️ [System Architecture](docs/architecture/system-architecture.md)

📡 [Communication Protocol](docs/architecture/protocol.md)

⚠️ [Failure Cases](docs/architecture/failure-cases.md)

---

## 🎯 Project Vision

The long-term vision is to evolve this first-generation demonstrator into a more capable emergency-robotics system with improved localization, communication, sensing, autonomous navigation and physical robustness.

The architecture is designed so that better hardware can replace simulated components without changing the fundamental system concept:

```text
Simulation
    ↓
Technology Demonstrator
    ↓
Physical Prototype
    ↓
Advanced Multi-Robot System
```

---

## 👥 Team

**Team:** [Team Name]

**IEEE ENIB Student Branch — Tunisia**

**TSYP14 Technical Challenge — The Living Map**

Developed for the **IEEE RAS × IEEE AESS Tunisia Section** technical challenge.

---

## 📌 Current Status

**Phase 1 — Software / Simulation Proof of Concept**

The repository is under active development as the project progresses toward the physical prototype.

---

### Core Idea

> **When the robot is gone, the knowledge should remain.**
