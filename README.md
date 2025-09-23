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
  - https://neurai.sites.northeastern.edu/research/wildlife%E2%80%91safe-backyard-patrol-drone/  

---
