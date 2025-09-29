# QADT AEAC 2026 Software Repository

The purpose of this repository is to house all software associated with development environments, containers, computer vision, autonomy, and more pertaining to the AEAC 2026 competition developed by **Queen's Aerospace Design Team (QADT)**.

# Development Environment Onboarding

The below sections cover onboarding setups required to setup the QADT AEAC 2026 development environment for **WSL**, **dual boot Ubuntu Linux**, and **MacOS**.

## Prerequisites

Learn about terminal development and navigating the linux shell.

## Onboarding - WSL

This project is developed and built inside Docker containers running on **WSL2 (Windows Subsystem for Linux)**. We will use **Ubuntu (latest)** as the default WSL distribution to keep the setup consistent across all contributors.

*In this guide, **WSL2** and **WSL** are used interchangeably; however, they both mean the same thing.*

The development workflow is:

1. **Run Docker inside WSL2 (via Docker Desktop).**
2. **Build and run project containers inside WSL2.**
3. **Attach VS Code to the running container** to edit and build code.
4. **Use Git inside WSL2** for source control.

### 1. Install WSL2 and Ubuntu

For reference, here's Microsoft’s official guide: [Install WSL](https://learn.microsoft.com/en-us/windows/wsl/install).

To install WSL, open any terminal in windows and enter the following:

```bash
wsl --install
```

Then, confirm that you have wsl installed by entering the following:

```bash
wsl --version
wsl --list --verbose
```

Make sure Ubuntu is installed and set as the **default distribution**:

```bash
wsl --set-default Ubuntu
```

Confirm this by entering `wsl --status` and you should see the following:

```bash
$ wsl --status
------------------------------
Default Distribution: Ubuntu
Default Version: 2
```

To run WSL, **press the windows key and search WSL** and launch the executable. Then run the following command:

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install Docker Desktop

Download and install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/). Docker desktop is a windows/mac application that will automatically detect running WSL instances and you'll be able to access **`docker`** commands within WSL without needing to install it explicitly.

During setup:
- Enable **WSL2 backend**.
- Integrate Docker Desktop with your **Ubuntu** WSL2 distribution.

After installation ensure that docker desktop is running on Windows. Then open a terminal in WSL and confirm Docker works:

```bash
docker --version
```

If this doesn't work, try restarting your environment by first quitting docker desktop, restarting WSL (press Cntrl + D in the WSL terminal to exit it), and try again.

### 3. Install Git in WSL

Since development happens inside of containers that we run from docker images, you’ll want Git available in your Ubuntu shell:

```bash
sudo apt update
sudo apt install -y git
```

Verify:

```bash
git --version
```

### 4. Clone the Repository

Inside WSL, make a `git` directory. This is where we will store the QADT AEAC 2026 repository. First navigate to your home (use `cd` to navigate directories, i.e `cd ~`) directory, make a `git` directory using `mkdir`. Then navigate into your git directory and clone the repo using `git clone`.

```bash
cd ~
mkdir git
cd git
git clone <your-repo-url>
```

### 5. Build and Run the Docker Container

From the project directory, build the image:

```bash
docker build -t project-dev .
```

Run the container (adjust ports/volumes as needed):

```bash
docker run -it --rm \
    -v $(pwd):/workspace \
    -p 3000:3000 \
    project-dev
```

### 6. Attach VS Code to Container

We use [VS Code Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

Steps:
1. Install VS Code and the **Remote - Containers** extension.
2. In WSL, run:
   ```bash
   code . # code (Visual Studio Code) is a command line executable that takes its first argument as 
          # The directory in which it will open up.
   ```
3. When VS Code opens, click the green button in the bottom-left corner:  
   **“Attach to Running Container”** → Select `project-dev`.

Now you can edit and build directly inside the container.

### 7. Workflow Summary

- Use **WSL2 (Ubuntu)** as your main environment.  
- **Git** operations happen in WSL.  
- **Docker Desktop** powers containers via WSL.  
- **VS Code Remote - Containers** attaches you to the running container for development.  

This ensures a consistent and reproducible environment across all contributors.

### Troubleshooting

- If Docker commands don’t work in WSL:
  - Ensure Docker Desktop is running.
  - Check Docker integration with Ubuntu in **Docker Desktop Settings → Resources → WSL Integration**.
- If `code .` doesn’t open VS Code:
  - Install [VS Code in Windows](https://code.visualstudio.com/Download).
  - Run `wsl --shutdown` in a **Windows terminal** and reopen Ubuntu.

## Onboarding - Dual Boot

[Install docker if not done already](https://docs.docker.com/engine/install/ubuntu/)

[Also setup managing docker as a non root user for ease of use](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

To allow X11 forwarding and see the UI for your session enter this command:

```bash
xhost +local:docker
```

You can also add it to profile to always make it active:

```bash
echo "xhost +local:docker > /dev/null" >> ~/.profile
```

Next step is to build the dockerfile into an image which can be done by navigating to QADT2026/Docker_Items and doing this command:

```bash
docker build -t qadt_image .
```

Then the image can be ran using the following. This is for the ***first run only***:
```bash
./run.sh
```

Other useful commands are listed here:
```bash
# Start the container
docker start -i qadt_cont

# Enter a running container from a new terminal
docker exec -it qadt_cont

# Stop a running container
docker stop qadt_cont

# Remove the container
docker rm qadt_cont
```

## Onboarding - MacOS

## Running ROS examples

Navigate to the `ros_examples_cpp` and enter the command:

```bash
colcon build && source install/setup.sh
```

To build all packages under the `AEAC2026/ros_examples_cpp/src` directory. Then run `ros2 run [pkg_name] [executable_name]`. For example, in one terminal instance run, `ros2 run service_02 server`, and in another terminal instance run, `ros2 run service_02 client`.

Feel free to experiment with the ROS2 examples.

# Software Executive Team

- **Max Dizy** $-$ *AEAC Co-Captain*
- **Kalena McCloskey** $-$ *Software Director*
- **Ethan Milburn** $-$ *Software Research (ICUAS) Manager*
- **Matthew Lones** $-$ *Computer Vision Manager*
- **Anthony Botticchio** $-$ *Autonomy Manager*
