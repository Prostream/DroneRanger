# Drone Ranger
_Backyard wildlife–safe patrol drone (ROS 2 · PX4 · Gazebo)_

**PI / Advisor:** **Yoon, Ilmi** (Northeastern University – Silicon Valley)

![drone-image](https://github.com/user-attachments/assets/b8e94ad4-c9ad-49e9-b498-fe2e88ef97f2)

---

## Overview
Urban wildlife is part of Bay Area life—but it can destroy gardens, attract predators, and create safety issues. Traditional fixes (traps, chemicals, tall fencing) are blunt, costly, or harmful. **NEURAI** is exploring a different approach: a small autonomous drone that politely discourages animals from entering sensitive areas using motion, light, and brief sound cues—**no contact and no harm**.

This project is led by **Prof. Ilmi Yoon** at **Northeastern University – Silicon Valley**. If you’d like to learn more or get involved, contact Prof. Yoon.

---

## What we’re building
A compact multirotor that patrols a geofenced backyard or campus area, detects common wildlife on-board, and runs short, humane deterrence behaviors. It logs what happened and why, so we can measure effectiveness and improve. The system is built **simulation-first** (Gazebo + ROS 2 + PX4), so students can test safely and then run the same code on real hardware.

---

## How it works
1. **Plan** — Define a geofence, privacy zones, and a patrol schedule.  
2. **Patrol** — The drone follows waypoints, loitering at key spots.  
3. **Detect** — On-board vision identifies wildlife; detections are confidence-checked.  
4. **Deter** — The drone performs a short flight/light/audio pattern, then stops.  
5. **Disengage & Log** — It returns to patrol or lands and records outcomes to guide improvements.

---

## Why it’s different
- **Humane by design** — No chasing or contact; short, bounded interactions.  
- **Privacy first** — Geofenced no-record zones and minimal data retention.  
- **Safety & compliance** — Manual override, return-to-home, and adherence to FAA/local rules.  
- **Research-ready** — A clean, modular stack (ROS 2 / PX4) for perception, planning, and controls research.  
- **Low-cost, open interfaces** — Works on accessible off-the-shelf hardware.

---

## Who benefits
- Homeowners and community gardens seeking humane deterrence.  
- Campus and facilities teams managing landscaped spaces.  
- Vineyards and small farms with crop-protection needs.  
- Students & researchers using a real-world autonomy testbed.

---

## Project status
- **Built:** Core simulator, offboard control, prototype wildlife detection.  
- **In progress:** Closed-loop deterrence logic, hardware bring-up, and backyard pilots.  
- **Next:** Field studies with partners and a public write-up on effectiveness.

---

## Safety & ethics
Short interactions only, no contact. Respect airspace and neighbors. Manual override available at all times. We consult wildlife and ethics guidance to align with humane best practices.

---

## Team Resources (curated from Teams)
Aggregated and categorized references for setup, simulation, and development.

### 1) Getting Started / Install
- **QGroundControl (QGC) — Download & Install (v5)**  
  <https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html>

### 2) Internal Notes & Guides
- **Senior Member Notes (internal GDoc)** — Setup tips, gotchas, and workflow guidance.  
  <https://docs.google.com/document/d/1o-dyEhTHNcyCs2oJularQTqOe9C_kmq4pDHSJ8qubyA/edit?tab=t.0>
- **Drone Project Setup & Progress (internal GDoc)** — Running log of project decisions and milestones.  
  <https://docs.google.com/document/d/1BMbUP61QGyJT_Y7DdSopzRrnQRWRtctJrLJMkWndU6I/edit?tab=t.0#heading=h.svnmdzcb7y41>

### 3) Simulation Logic Diagram

  <img width="1804" height="543" alt="image" src="https://github.com/user-attachments/assets/871433ff-a095-4967-b88b-b073ebe14cbf" />


### 4) Key Repos & Middleware
- **PX4 Autopilot** — Flight stack for SITL/HITL and hardware.  
  <https://github.com/PX4/PX4-Autopilot>
- **Micro XRCE-DDS Agent (eProsima)** — Lightweight DDS agent for microcontrollers / companion links.  
  <https://github.com/eProsima/Micro-XRCE-DDS-Agent>

### 5) Companion Computer & Integration
- **PX4 ↔ Raspberry Pi (Companion Computer)** — Messaging and integration guide.  
  <https://docs.px4.io/main/en/companion_computer/pixhawk_rpi>

### 6) Simulation & Robotics Frameworks
- **NVIDIA Isaac Sim** — Physically‑based sim on Omniverse; sensors & USD workflows.  
  <https://github.com/isaac-sim/IsaacSim>
- **Drake (MIT)** — Modeling, simulation, and control with modern tooling.  
  <https://drake.mit.edu/>

### 7) Compliance & Remote ID
- **PX4 Remote ID** — Reference for Remote ID support and hardware options.  
  <https://docs.px4.io/main/en/peripherals/remote_id.html>

---

## Quick Links
- NEURAI project page:  
  https://neurai.sites.northeastern.edu/research/wildlife%E2%80%91safe-backyard-patrol-drone/
- PX4 Autopilot repo:  
  https://github.com/PX4/PX4-Autopilot
- QGC install guide (v5):  
  https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html

---

