# FRC_2022
AEMBOT 6443's 2022 FRC Repository

## Requirements
To build this repository and to push to the robot, FRC tools including wpilib are required.
1. Download and install the [FRC Game Tools](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#440024) from NI.
2. Following the [guide provided by FIRST](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html), download and install [wpilib](https://github.com/wpilibsuite/allwpilib/releases/tag/v2022.2.1) from GitHub.
3. Install git (Full instructions [here](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)):
  * On Windows, download and install git using the [official link](https://git-scm.com/download/win)
  * On most versions of Linux, git is included with the operating system. If not, you can install it with your package manager:
    * For Debian-based distributions: `sudo apt install git-all`
    * For RPM-based distributions: `sudo dnf install git-all`
  * On macOS, the best practice for installing git is to use the Xcode Command Line Tools. Run any git command in a Terminal (eg. `git --version`), and if git is missing, you will be prompted to install it.

## Setup
1. Clone this repository:
You can use VSCode's builtin git tools to do this, or the command line:
`git clone https://github.com/LibertyRobotics/FRC_2022`
2. To build and deploy the project, use the VSCode keyboard shortcut <kbd>Shift</kbd> + <kbd>F5</kbd>, or access the `Command Palette` (<kbd>Ctrl</kbd> +<kbd>Shift</kbd> + <kbd>P</kbd>) and search for the `Deploy` Command.

## About this Project
AEMBOT uses the WPILIB's Command-Based Programming pattern for implementing robot code. In this paradigm, robot sensors and actuators are grouped into subsystems, which expose an interface to Commands, which define the behaviors and states of the robot. Understanding the Command-based model is crucial to understanding this project. Learn more about Command-Based Programming [here](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html).

## Key Bindings

| Button  | Action | Details |
| -- | -- | -- |
| **Primary Controller** | -- | -- |
| Joysticks | Arcade Drive | Currently set to lower power for Girls Gen and new user training |
| A | Turn to Hub | Requires hub to be in sight, will buzz if not |
| Start | Advance climb sequence | |
| **Secondary Controller** | -- | -- |
| A | Run intake roller belts | |
| X | Run intake roller belts in reverse | |
| B | Ramp up and advance shooter/indexer | |
| Left/Right Bumper | Raise/lower intake | No need to hold down |
| DPad up/down | Manually increment/decrement shooter RPM offset | |


## Resources
* [FRC Control System Docs](https://docs.wpilib.org/en/stable/index.html)
* [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
* [wpilibj javadocs](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/index.html)

![Robots. Don't. Quit.](https://drive.google.com/uc?export=download&id=1FHWfhFtUOuyOHyrDigHGzEt0eMrn_GTo#gh-dark-mode-only)
![Robots. Don't. Quit.](https://user-images.githubusercontent.com/31430937/163511327-a63c9b28-d52f-478e-8b6b-478de8449ab4.png#gh-light-mode-only)
