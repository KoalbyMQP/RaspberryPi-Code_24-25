# RaspberryPi-Code_23-24
## Raspberry Pi Side of Koalby/Ava

### Install and Use
1. Clone GitHub repo
    1. https://github.com/KoalbyMQP/RaspberryPi-Code_23-24
2. Switch to the dev branch of the repository
3. Install all Python packages in requirements.txt in the root folder of the repository
    1. pip install -r requirements.txt
4. Download and install latest version of CoppeliaSim for your OS
    1. https://coppeliarobotics.com/
5. Open CoppeliaSim
    1. For Ubuntu and Mac OSs, open CoppeliaSim through terminal
6. Open wanted scene in CoppeliaSim
    1. Our scenes are stored in RaspberryPi-Code_23-24/backend/KoalbyHumanoid/Simulation Files
    2. File→Open scene...→ (select the scene file)→ Open
7. Run the Python script corresponding to what you want to happen in the scene
    1. Our demo scripts are stored in RaspberryPi-Code_23-24/backend/Demo Scripts
    2. Make sure to run any scripts locally and from the root folder of the repository

### Programs
- KoalbyHumanoid
  - Simulation Files
    - Finly_Updated_URDF.ttt - CoppeliaSim model of Finley
    - gyro.lua - code for gyroscope sensor in CoppeliaSim
  - Config.py: Robot configuration (links and motors)
  - Motor.py: RealMotor and SimMotor classes to get/set motor positions
  - PID.py: PID class to calculate output based on error
  - Plotter.py: plots points in 3D space using matplotlib
  - poe.py: Product-of-Exponentials code (https://en.wikipedia.org/wiki/Product_of_exponentials_formula)
  - Robot.py: RealRobot and SimRobot classes to control the entire robot
  - trajPlanner.py: Basic Trajectory Planner (like from RBE 3001)
- Simulation
  - sim.py: python API to CoppeliaSim
  - simConst.py: CoppeliaSim constants
  - simpleTest.py: Simple script to test connection between python and CoppeliaSim (current non-functional)
- Testing: various test scripts
  - initRobot: initializes a RealRobot object
  - initSim: initalizes the simulation (connects python to CoppeliaSim)
  - simMotionTest: testing balancing while standing and bending over
  - trajplannertest: tests the trajPlanner class in the KoalbyHumanoid package

## Branches (Update as new branches are made)
### main
Ready to deploy for good production codebase on Ava's Raspberry Pi
### dev
Ready to test development codebase on Ava's Raspberry Pi
### sim
Simulation development codebase
