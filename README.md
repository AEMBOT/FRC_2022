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


## Resources
* [FRC Control System Docs](https://docs.wpilib.org/en/stable/index.html)
* [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
* [wpilibj javadocs](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/index.html)

![Robots](https://drive.google.com/uc?export=download&id=1FHWfhFtUOuyOHyrDigHGzEt0eMrn_GTo#gh-dark-mode-only)
![image](https://user-images.githubusercontent.com/6174676/151886133-8e62bf21-fdb9-42f0-bb85-9702b77ee404.png)
