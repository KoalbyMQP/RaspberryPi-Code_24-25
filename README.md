# RaspberryPi-Code_23-24
## Raspberry Pi Side of Koalby/Ava

### Install and Use
1. Download and install the latest versions of CoppeliaSim, Git, and Python for your OS
    1. CoppeliaSim: https://coppeliarobotics.com/
    2. Git: https://git-scm.com/downloads/
    3. Python: https://www.python.org/downloads/
2. Clone this GitHub repository
    1. https://github.com/KoalbyMQP/RaspberryPi-Code_23-24
3. Install all Python packages in requirements.txt found in the root folder of this repository. Newer versions of pip discourage installing packages outside of a virtual environment, so --break-system-packages is appended to ensure that the packages have full access to the system.
    1. pip install -r requirements.txt --break-system-packages
4. Open CoppeliaSim
    1. For Ubuntu and macOS, you may have to open CoppeliaSim through terminal if not working
5. Open wanted scene in CoppeliaSim
    1. Our scenes are located at RaspberryPi-Code_23-24/backend/KoalbyHumanoid/Simulation Files
    2. In CoppeliaSim: File → Open scene... → (select the scene file) → Open
6. Run the Python script corresponding to what you want to happen in the scene
    1. Each test file is in the corresponding titled folder in RaspberryPi-Code_23-24/backend
           1.1 Note: Most calculations and functions referenced will be found in files within RaspberryPi-Code_23-24/backend/KoalbyHumanoid
           1.2 Note: Any outdated files to view members' initial code or thought process are located at RaspberryPi-Code_23-24/backend/Archive
    3. Make sure to run any scripts locally and from the root folder of this repository

### Programs
- KoalbyHumanoid
  - Simulation Files
    - Finly_T.ttt or Koalby_T.ttt - CoppeliaSim model of Finley used for balancing related testing
    - Finly_Walking_Assisted.ttt - CoppeliaSim model of Finley with replicated cart model for assisted walking testing
    - gyro.lua - code for gyroscope sensor in CoppeliaSim
  - Config.py: Robot configuration (links and motors)
  - Motor.py: RealMotor and SimMotor classes to get/set motor positions
  - PID.py: PID class to calculate output based on error
  - Plotter.py: plots points in 3D space using matplotlib
  - poe.py: Product-of-Exponentials code (https://en.wikipedia.org/wiki/Product_of_exponentials_formula)
  - Robot.py: RealRobot and SimRobot classes to control the entire robot
  - trajPlanner.py: Basic Trajectory Planner (like from RBE 3001)
- MATLAB Scripts
  - Forward and Inverse Kinematic Trajectories for balancing and assisted walking
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

## Past Branches: All previous branches a team member worked on has been combined into main. For clarification on any specific code: 
1. On the right hand side of the repository click Activity
2. Click All Activity and change to Branch Deletions
3. All outdated, previous branches will appear and are named according to what concept/test was focused on in the code. To properly view, click the three dots on the right side of the branch name to restore branch.
