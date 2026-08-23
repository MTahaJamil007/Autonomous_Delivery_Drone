# References

**Verification convention.**
`[URL-2026-08-23]` = the URL was returned by a literature search on 2026-08-23
and the title/claim is as reported by that search. It has **not** been read in
full — do that before citing in a submission.
`[STD]` = standard/foundational reference cited from domain knowledge; look up
the exact bibliographic details before submission.

---

## Precision landing and fiducial markers

| Key | Reference | Status |
| --- | --- | --- |
| `Springer-JIRS-2025` | *Vision-Based Autonomous UAV Landing: A Comprehensive Review of Technologies, Techniques, and Applications*, J. Intelligent & Robotic Systems, 2025. Reviews 143 papers, 2018–2025. https://link.springer.com/article/10.1007/s10846-025-02314-4 | URL-2026-08-23 |
| `RArUco-2026` | *Recursive ArUco Markers: A Scalable Fiducial Marker Design for UAV Landing Pads*, arXiv:2607.13830. 100% detection to 30% occlusion. https://arxiv.org/html/2607.13830 | URL-2026-08-23 |
| `Embedded-ArUco-2021` | *Embedded ArUco: a novel approach for high precision UAV landing*, IEEE, 2021. 2.03 cm mean error (σ 1.53 cm). https://ieeexplore.ieee.org/document/9438855/ | URL-2026-08-23 |
| `Electronics-2026-Fiducial` | *Experimental Evaluation of Precision Positioning in Unmanned Aerial Systems Using Fiducial Markers*, Electronics. ArUco vs AprilTag detection rate/throughput. https://doi.org/10.3390/electronics15081582 | URL-2026-08-23 |
| `Springer-IJASS-2023` | *Fiducial Marker-Based Autonomous Landing Using Image Filter and Kalman Filter*, Int. J. Aeronautical & Space Sciences, 2023. https://link.springer.com/article/10.1007/s42405-023-00635-y | URL-2026-08-23 |
| `Sim2Real-DRL-Landing-2024` | *Closing the Sim-to-Real Gap: Enhancing Autonomous Precision Landing of UAVs with Detection-Informed Deep Reinforcement Learning*, Springer, 2024. https://link.springer.com/chapter/10.1007/978-3-031-66694-0_11 | URL-2026-08-23 |
| `TornadoDrone-2024` | *TornadoDrone: Bio-inspired DRL-based Drone Landing on 6D Platform with Wind Force Disturbances*, arXiv:2406.16164 | URL-2026-08-23 |
| `RiskAware-EmergencyLanding-2026` | *Vision-Based Risk Aware Emergency Landing for UAVs in Complex Urban Environments*, arXiv:2505.20423 / ScienceDirect S095219762601105X | URL-2026-08-23 |
| **`Evidence-Landing-2026`** | ***Evidence-Based Landing Site Selection and Vision-Based Landing for UAVs in Unstructured Environments**, arXiv:2605.01432. Latent landing-safety variable, recursive temporal accumulation of visual cues, decision separated from visual-servo execution.* **CLOSEST COMPETITOR TO D2 — read in full.** https://arxiv.org/html/2605.01432 | URL-2026-08-23 |
| `NeuroSymbolic-Landing-2025` | *Human-Inspired Neuro-Symbolic World Modeling and Logic Reasoning for Interpretable Safe UAV Landing Site Assessment*, arXiv:2510.22204 | URL-2026-08-23 |
| `Gazebo-Fiducial-Testing-2023` | *Testing Procedures Architecture for Establishing a Fiducial Marker Recognition Quality in UAV-based Visual Marker Tracking Task in Gazebo Simulator*, 2023. ResearchGate 367771547 | URL-2026-08-23 |
| `ArUcoNano-2026` | *ArUco Nano: a simpler, faster, and more reliable fiducial marker detector*, SoftwareX. https://www.sciencedirect.com/science/article/pii/S2352711026001822 | URL-2026-08-23 |
| `iMarkers-2025` | *Unveiling the Potential of iMarkers: Invisible Fiducial Markers for Advanced Robotics*, arXiv:2501.15505 | URL-2026-08-23 |
| `Garrido-Jurado-2014` | Garrido-Jurado et al., *Automatic generation and detection of highly reliable fiducial markers under occlusion*, Pattern Recognition 47(6), 2014. The ArUco paper. | STD |
| `Olson-2011` | E. Olson, *AprilTag: A robust and flexible visual fiducial system*, ICRA 2011. | STD |

## Mapping, shared memory, evidence

| Key | Reference | Status |
| --- | --- | --- |
| `Bosch-Evidential-2024` | *Accurate Training Data for Occupancy Map Prediction in Automated Driving Using Evidence Theory*, CVPR 2024. https://github.com/boschresearch/evidential-occupancy | URL-2026-08-23 |
| `Evidential-Road-2021` | *Fusion of neural networks for LIDAR-based evidential road mapping*, arXiv:2102.03326 | URL-2026-08-23 |
| `OGM-Merging` | *Occupancy Grid Map Merging for Multiple Robot Simultaneous Localization and Mapping*. Merge by greater absolute magnitude, not sum, to avoid double-counting. | URL-2026-08-23 |
| `NGASAC-2025` | *Multi-UAV Cooperative Search in Partially Observable Low-Altitude Environments Based on Deep Reinforcement Learning*, Drones 9(12):825, 2025. https://www.mdpi.com/2504-446X/9/12/825 | URL-2026-08-23 |
| `Dual-Timescale-MADDPG-2025` | *Dual-timescale hierarchical MADDPG for Multi-UAV cooperative search*, JKSU-CIS, 2025. https://link.springer.com/article/10.1007/s44443-025-00156-6 | URL-2026-08-23 |
| `ARCog-NET-2025` | *Implementation of monocular visual SLAM with ARCog-NET for aerial robot swarm indoor mapping*, Scientific Reports, 2025. Edge–Fog–Cloud collective knowledge reuse. https://www.nature.com/articles/s41598-025-28618-x | URL-2026-08-23 |
| `Swarm-CA-Survey-2025` | *Collision avoidance in UAV swarms: A learning-centric perspective on collaborative intelligence*, ScienceDirect S092523122502692X | URL-2026-08-23 |
| `Moravec-Elfes-1985` | Moravec & Elfes, *High resolution maps from wide angle sonar*, ICRA 1985. Origin of occupancy grids. | STD |
| `Elfes-1989` | A. Elfes, *Using occupancy grids for mobile robot perception and navigation*, IEEE Computer 22(6), 1989. | STD |
| `Thrun-2005` | Thrun, Burgard & Fox, *Probabilistic Robotics*, MIT Press 2005. Ch. 9: occupancy grid mapping, inverse sensor models, log-odds. | STD |
| `Pagac-1998` | Pagac, Nebot & Durrant-Whyte, *An evidential approach to map-building for autonomous vehicles*, IEEE T-RA 14(4), 1998. | STD |
| `Grossberg-1987` | S. Grossberg, *Competitive learning: from interactive activation to adaptive resonance*, Cognitive Science 11, 1987. Stability–plasticity dilemma. | STD |

## Arbitration, replanning, reactive avoidance

| Key | Reference | Status |
| --- | --- | --- |
| **`When2Replan-2023`** | ***When to Replan? An Adaptive Replanning Strategy for Autonomous Navigation using Deep Reinforcement Learning**, OMRON SINIC X. Learned replan-timing vs periodic and patience-timer baselines, 100 trials/layout.* **CLOSEST COMPETITOR TO D3 — read in full.** https://omron-sinicx.github.io/when2replan/ | URL-2026-08-23 |
| `Drones-2025-OA-Survey` | *A Survey on Obstacle Detection and Avoidance Methods for UAVs*, Drones 9(3):203, 2025. Reactive vs deliberative taxonomy. https://doi.org/10.3390/drones9030203 | URL-2026-08-23 |
| `DRPA-MPPI-2025` | *DRPA-MPPI: Dynamic Repulsive Potential Augmented MPPI for Reactive Navigation in Unstructured Environments*, arXiv:2503.20134 | URL-2026-08-23 |
| `SA-APF-2025` | *A Multi-UAV Formation Obstacle Avoidance Method Combining Improved Simulated Annealing and Adaptive Artificial Potential Field*, arXiv:2504.11064 / Drones 9(6):390 | URL-2026-08-23 |
| `RealTime-OA-2025` | *Real-Time Obstacle Avoidance Algorithms for Unmanned Aerial and Ground Vehicles*, arXiv:2506.20311 | URL-2026-08-23 |
| `InFeR-2025` | *InFeR: Informed Failure Resilience in Learned Visual Navigation Control*, arXiv:2510.24680 | URL-2026-08-23 |
| `Borenstein-Koren-1991` | Borenstein & Koren, *The vector field histogram — fast obstacle avoidance for mobile robots*, IEEE T-RA 7(3), 1991. Also *Potential field methods and their inherent limitations* (1991) for the local-minima result. | STD |

## Perception-aware and informative planning

| Key | Reference | Status |
| --- | --- | --- |
| `RAPTOR-2021` | *RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight*, arXiv:2007.03465 / T-RO | URL-2026-08-23 |
| `PAP-FeatureLimited-2025` | *Perception-aware Planning for Quadrotor Flight in Unknown and Feature-limited Environments*, arXiv:2503.15273 | URL-2026-08-23 |
| `FLAP-2026` | *FLAP: FOV-Constrained Active Perception Planning for Prior-Map-Free 3D Navigation*, arXiv:2606.17630 | URL-2026-08-23 |
| `Visibility-Tracking-2021` | *Visibility-aware Trajectory Optimization with Application to Aerial Tracking*, arXiv:2103.06742 | URL-2026-08-23 |
| `StarConvex-2022` | *Star-Convex Constrained Optimization for Visibility Planning with Application to Aerial Inspection*, arXiv:2204.04393 | URL-2026-08-23 |
| `DualControl-2025` | *Implicit Dual-Control for Visibility-Aware Navigation in Unstructured Environments*, arXiv:2507.04371 | URL-2026-08-23 |
| `Obstacle-Aware-IPP-2019` | *Obstacle-aware Adaptive Informative Path Planning for UAV-based Target Search*, ICRA 2019. ResearchGate 331396680 | URL-2026-08-23 |
| `IPP-TerrainMonitoring` | *An informative path planning framework for UAV-based terrain monitoring*. Altitude trades FOV against sensor noise. ResearchGate 339030424 | URL-2026-08-23 |
| `SPOT-2025` | *SPOT: Sensing-augmented Trajectory Planning via Obstacle Threat Modeling*, arXiv:2510.16308 | URL-2026-08-23 |

## Safety, assurance, testing, degradation

| Key | Reference | Status |
| --- | --- | --- |
| `Aerialist-ICSE-2024` | *Simulation-based Testing of Unmanned Aerial Vehicles with Aerialist*, ICSE 2024 Companion. https://dl.acm.org/doi/10.1145/3639478.3640031 | URL-2026-08-23 |
| `DroneTestPipeline-2025` | *A Step-by-Step Guide to Creating a Robust Autonomous Drone Testing Pipeline*, arXiv:2506.11400 | URL-2026-08-23 |
| `sUAS-Fuzzing-2026` | *Uncovering Failures in Cyber-Physical System State Transitions: A Fuzzing-Based Approach Applied to sUAS*, arXiv:2601.05449 | URL-2026-08-23 |
| `REDriver-2024` | *REDriver: Runtime Enforcement for Autonomous Vehicles*, arXiv:2401.02253. LTL specifications. | URL-2026-08-23 |
| `NASA-RTA-2024` | *A Verification Framework for Runtime Assurance of Autonomous UAS*, NASA LaRC, DASC 2024. https://shemesh.larc.nasa.gov/fm/papers/DASC2024-SWDMC-draft.pdf | URL-2026-08-23 |
| `Drones-RuleBased-2024` | *Rule-Based Verification of Autonomous Unmanned Aerial Vehicles*, Drones 8(1):26, 2024. | URL-2026-08-23 |
| `MissionRTA-2026` | *Mission-Level Runtime Assurance Framework for Autonomous Driving*, arXiv:2606.06996 | URL-2026-08-23 |
| `Uncertainty-Unsafety-2025` | *When uncertainty leads to unsafety: Empirical insights into the role of uncertainty in UAV safety*, Empirical Software Engineering, 2025. PX4-integrable uncertainty monitors. https://link.springer.com/article/10.1007/s10664-025-10697-z | URL-2026-08-23 |
| `BASiC-2024` | *UAV sensor failures dataset: Biomisa ArduCopter Sensory Critique (BASiC)*, Data in Brief. PMC10831493 | URL-2026-08-23 |
| `SimToReal-Testbed-2026` | *A UAV Testbed for Diagnosing Hardware Vulnerabilities: Quantifying Sim-to-Real Discrepancies in PX4 Flight Logs*, Sensors. https://doi.org/10.3390/s26103188 | URL-2026-08-23 |
| `RiskMDP-Contingency-2023` | *Risk-Aware MDP Contingency Management Autonomy for Uncrewed Aircraft Systems*, J. Aerospace Information Systems. https://arc.aiaa.org/doi/10.2514/1.I011235 | URL-2026-08-23 |

## Energy, payload, routing

| Key | Reference | Status |
| --- | --- | --- |
| `LPED-BatteryAware-2018` | *Battery-Aware Energy Model of Drone Delivery Tasks*, ISLPED 2018. https://dl.acm.org/doi/10.1145/3218603.3218614 | URL-2026-08-23 |
| `PayloadMass-Trajectory-2020` | *Payload-Mass-Aware Trajectory Planning on Multi-User Autonomous UAVs*, arXiv:2001.02531 | URL-2026-08-23 |
| `EnergyAwareMCPP-2024` | *Energy-aware Multi-UAV Coverage Mission Planning with Optimal Speed of Flight*, CTU MRS. https://github.com/ctu-mrs/EnergyAwareMCPP | URL-2026-08-23 |
| `Drones-MultiTrip-2026` | *Energy Consumption Optimization of Multi-Trip UAV Routing Using Surrogate Modeling*, Drones 10(6):430. https://doi.org/10.3390/drones10060430 | URL-2026-08-23 |
| `EnergyAware-Safe-2025` | *Energy Aware and Safe Path Planning for Unmanned Aircraft Systems*, arXiv:2504.03271 | URL-2026-08-23 |
| `Hydrogen-Review-2025` | *A comprehensive review and future challenges of energy-aware path planning for small UAVs with hydrogen-powered hybrid propulsion*, The Aeronautical Journal, CUP. | URL-2026-08-23 |
| `BatteryHealth-2026` | *Motion-Specific Battery Health Assessment for Quadrotors Using High-Fidelity Battery Models*, arXiv:2603.12791 | URL-2026-08-23 |

## GNSS-denied navigation

| Key | Reference | Status |
| --- | --- | --- |
| `SatNav-GNSSDenied-2025` | *GNSS-denied UAV navigation: analyzing computational complexity, sensor fusion, and localization methodologies*, Satellite Navigation, 2025. https://link.springer.com/article/10.1186/s43020-025-00162-z | URL-2026-08-23 |
| `FMC-SVIL-2023` | *UAV navigation in large-scale GPS-denied bridge environments using fiducial marker-corrected stereo visual-inertial localisation*, Automation in Construction, 2023. ~50% pose-error reduction. | URL-2026-08-23 |
| `SCLAM-2025` | *A Simultaneous Control, Localization, and Mapping System for UAVs in GPS-Denied Environments*, Drones 9(1):69, 2025. | URL-2026-08-23 |
| `Heightmap-SPRIND-2025` | *Kilometer-Scale GNSS-Denied UAV Navigation via Heightmap Gradients: A Winning System from the SPRIN-D Challenge*, arXiv:2510.01348 | URL-2026-08-23 |

## Learning-based navigation

| Key | Reference | Status |
| --- | --- | --- |
| `MARL-UAV-Survey-2025` | *A Survey on UAV Control with Multi-Agent Reinforcement Learning*, Drones 9(7):484, 2025. | URL-2026-08-23 |
| `Coop-MARL-2026` | *Cooperative Multi-UAV Navigation in Complex Environments via Systematic Multi-Agent Deep Reinforcement Learning*, arXiv:2607.25754 | URL-2026-08-23 |
| `MemSAC-2024` | *Memory-based soft actor–critic with prioritized experience replay for autonomous navigation*, Intelligent Service Robotics, 2024. | URL-2026-08-23 |
| `LLfN-2020` | *A Lifelong Learning Approach to Mobile Robot Navigation*, arXiv:2007.14486 / RA-L. | URL-2026-08-23 |

## Empirical software studies

| Key | Reference | Status |
| --- | --- | --- |
| `FSE21-Autopilot-Bugs` | *An Exploratory Study of Autopilot Software Bugs in Unmanned Aerial Vehicles*, ESEC/FSE 2021. 168 UAV-specific bugs from 569; 19.6% misconfigurations. https://yuleisui.github.io/publications/fse21.pdf | URL-2026-08-23 |
| `ROBUST-2024` | *ROBUST: 221 Bugs in the Robot Operating System*, arXiv:2404.03629 / EMSE. | URL-2026-08-23 |
| `ROS-InteractionBugs-2025` | *An Empirical Study of Interaction Bugs in ROS-based Software*, arXiv:2507.10235 | URL-2026-08-23 |
| `ROS-Misconfig-2024` | *Understanding Misconfigurations in ROS: An Empirical Study and Current Approaches*, arXiv:2407.19292 | URL-2026-08-23 |
| `CPS-Bugs-2022` | *An empirical characterization of software bugs in open-source Cyber–Physical Systems*, JSS, 2022. | URL-2026-08-23 |
