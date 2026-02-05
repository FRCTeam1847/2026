# FRC 1847 2025 Robot Code

This repository contains the code for FRC Team 1847’s 2025 robot.

## Table of Contents
- [Setup](#setup)
  - [Required Tools](#required-tools)
  - [Helpful Tools](#helpful-tools)
- [Cloning the Repository](#cloning-the-repository)
- [Running the Code](#running-the-code)
  - [Deploying to a Physical Robot](#deploying-to-a-physical-robot)
  - [Running in Simulation](#running-in-simulation)
    - [Starting the Simulation](#starting-the-simulation)
    - [Using the FRC Driver Station](#using-the-frc-driver-station)
    - [Using AdvantageScope](#using-advantagescope)

---

## Setup

### Required Tools
1. **WPILib + VS Code**  
   - Follow the [WPILib installation guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) to set up VS Code with the WPILib extension.  
   - Install the [FRC Game Tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html) (Driver Station, etc.).

### Optional Tools
- **PathPlanner**  
  A graphical tool for creating autonomous paths. You can download it from the [Microsoft Store](https://apps.microsoft.com/detail/9nqbkb5dw909).

---

## Cloning the Repository
To edit and run this code, you will need to clone this repository locally. Assuming you have installed WPILib + VS Code:

1. Open **2026 WPILib VS Code**.
2. Click **Clone Git Repository** (found in the welcome screen or from the Command Palette).
3. When prompted, enter the repository URL: `https://github.com/FRCTeam1847/2026.git`
4. Choose a folder to save the repository locally. (remember this location you will need it later)
5. When prompted to open the cloned repository, select **Open**.
    - If prompted about trusting the authors, select **Yes** (you must trust the workspace to run code).

---

## Running the Code
There are 2 main ways to run this code from VS Code: deploying to a physical robot or running a simulator.

### Deploying to a Physical Robot
This method requires your development machine to be connected to the robot (via USB, Ethernet, or Wi-Fi).

1. In VS Code:
1. Open the Command Palette by pressing <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> (Windows/Linux) or <kbd>Cmd</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> (macOS).
2. Type `Deploy Robot Code` and select **WPILib: Deploy Robot Code**.
3. Wait for the `Build Succeeded` message in the terminal.
2. Open the **FRC Driver Station**. You should see **Robot Code** turn green once the code has successfully deployed.

---

### Running in Simulation with AdvantageScope
Running your code in simulation allows you to test robot logic without a physical robot.

#### Starting the Simulation
1. In VS Code:
1. Open the Command Palette (<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>).
2. Type `Simulate Robot Code` and select **WPILib: Simulate Robot Code**.
3. Wait for the `Build Successful` message.
4. In the pop-up, choose **Use Real DriverStation**.

#### Using the FRC Driver Station
- Open the **FRC Driver Station** on your computer.
- It should connect to the simulated robot automatically if you chose **Use Real DriverStation**.

#### Using AdvantageScope
AdvantageScope provides real-time visualization and data logging:

In VS Code:
1. Open the Command Palette (<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>).
2. Type `Start Tool` and select **WPILib: Start Tool**.
3. Wait for AdvantageScope to open.

Inside AdvantageScope:
1. Press <kbd>Alt</kbd>+<kbd>3</kbd> or click the **`+`** icon (top right) to open **3D Field**.
2. Click **File** → **Connect to Simulator** → **Default: NetworkTables4** (or press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>K</kbd>).
3. Under **Field** (on the botom right), ensure **2026 Field** is selected.
4. **Add the Robot to the Field**:  
   - In the left panel, expand `AdvantageKit` → `Field`.
   - Click and drag `Robot-Pose3d` into the `Poses` section.
   - Click and drag `Turret-Pose3d` onto the `Robot-Pose3d` in the the `Poses` section.
5. **Use Team’s Custom CAD Assets** (optional):
   - Click **App** in the menu bar and select **Use Custom Assets Folder**.
   - Navigate to this project’s `assets` folder and click `Open`.
   - In the `Poeses` section click on the arrow icon next to `2026 KitBot`. In the menu click `1847-WRATH`
   - Wait for the assets to load. You should now see your team’s custom robot model on the field.
---

