# Self-Diagnosis Toolkit: MSCA Project submodule

[![EU Funding](https://img.shields.io)](https://aigreenbots.eu)

This submodule contains the software implementation for the **AIGreenBots** adaptive, run-time self-diagnosis toolkit designed for field robotics.

---

## 📺 Project Demonstrations

#### 🔹 1. USV Degradation Detection in Aquatic Environment
https://github.com/user-attachments/assets/7771c56a-42a1-4d3e-99e2-2dfcfdc89d7a

#### 🔹 2. USV Intermittent Fault Detection in Aquatic Environment
https://github.com/user-attachments/assets/e65c580a-da22-44be-87c8-0bbafb7e7049

---

## Repository Structure (primary folders)

```
sequential-vs-static-fault-detection-swarm/
├── remora_controller/   # Package containing nodes and launch files for running control loops for the USV simulator
└── fault_detector/      # Packe containing fault detection nodes (LSTM and GBDT)
```

---

## Dependencies

**ROS2:** Jazzy  
**Python:** 3.12.3  

| Package | Version |
|---------|---------|
| tensorflow | 2.20.0 |
| scikit-learn | 1.3.2 |
| joblib | 1.5.2 |
| numpy | 1.26.4 |
| pandas | 2.3.3 |
| psutil | 7.1.0 |

---

## 🇪🇺 Acknowledgments
This project has received funding from the European Union's Horizon Europe research and innovation programme under the Marie Skłodowska-Curie Actions (MSCA) Doctoral Network, under grant agreement No **101169330**. Views and opinions expressed are those of the author(s) only and do not necessarily reflect those of the European Union or the European Research Executive Agency (REA).

---
© 2026 Faisal Mazloum — AIGreenBots Project
