# kOS Autonomous Rocket Guidance & Booster Scripts (Kerbal Space Program)

A collection of advanced autonomous flight control, guidance, and landing scripts written in **kOS (Kerbal Operating System)**, featuring implementations of historical aerospace guidance algorithms for precision rocket recovery and planetary landings.

## 🚀 Key Features

* **Apollo Powered Descent Guidance (APDG):** Implementation of the classic Apollo Lunar Module guidance laws (`apdg.ks`) for targeted velocity-to-be-gained steering and optimal throttle profiles during descent.
* **Autonomous Boostback Guidance:** Real-time trajectory error correction and predictive targeting (`boostback/`) for precise booster return maneuvers.
* **Numerical Trajectory Integration:** Custom simulation engines (`prediction/`) utilizing numerical integration to model atmospheric drag, gravity turns, and landing trajectories.
* **Automated Landing Burn Control:** Closed-loop suicide burn and terminal velocity management algorithms (`landingburn.ks`).
* **Aerodynamic Descent Integration:** Atmospheric density and airbrake deployment tracking (`aerodescent.ks`, `airbrakeForce.ks`).

## 🛠️ Tech Stack & Concepts

* **Language:** kOS Script (Kerbal Operating System / Parsed DSL)
* **Domain:** Aerospace Engineering, Guidance, Navigation, and Control (GNC)
* **Algorithms:** Apollo Powered Descent Guidance (APDG), numerical integration, PID error correction, iterative projection targeting, state estimation

<img width="480" height="270" alt="output" src="https://github.com/user-attachments/assets/90a4bef2-ecfc-487e-bf80-a1ca2bb65805" />
