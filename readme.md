# QADT AEAC 2026 Software Repository

The purpose of this repository is to house all software associated with development environments, containers, computer vision, autonomy, and more pertaining to the AEAC 2026 competition developed by **Queen's Aerospace Design Team (QADT)**.

---

# Development Environment Setup

The below sections cover onboarding setups required to setup the QADT AEAC 2026 development environment for **WSL**, **dual boot Ubuntu Linux**, and **MacOS**.

---

## Prerequisite Learning Content

***You may refer to the resources in this section as you complete your onboarding.***

Learn some of the basics of **Linux** and navigating the terminal, **Docker**, and **Git**. Prioritize learning about Linux and complete the first few onboarding tasks until you reach the docker section. Review docker, and optionally you may begin learning git. Learning the basics of git will be ***necessary*** for future contributions to QADT Software. 


1. **Linux Fundamentals** video. Click on the link in the description to go to the website seen in the video.
[![HERE](https://img.youtube.com/vi/GDPjY7DKpSQ/maxresdefault.jpg)](https://www.youtube.com/watch?v=GDPjY7DKpSQ)
2. You may *optionally* learn a bit about **Docker** since you'll be using it in the following sections.
[![HERE](https://img.youtube.com/vi/DQdB7wFEygo/maxresdefault.jpg)](https://www.youtube.com/watch?v=DQdB7wFEygo)
3. Learn fundamentals of **git** version control with [this](https://www.w3schools.com/git/) W3schools tutorial.

---

## Prerequisite Setup 

**This step is REQUIRED for all operating systems.**

## Install VSCode

Install VSCode for your operating system from **[this link for Windows and MacOS](https://code.visualstudio.com/download)** and **[this link for Linux](https://code.visualstudio.com/docs/setup/linux)**. We will be using the **Dev Containers** VSCode extension to download Docker images and open development containers inside VSCode.

### Create a GitHub Account

If you don’t already have a GitHub account:

1. Go to [https://github.com/signup](https://github.com/signup)
2. Follow the steps to create an account (I recommend that you create it under a personal email)
3. Verify your email address when prompted

Once your account is created, **sign in** and keep the page open — you’ll need it shortly when setting up SSH access and cloning repositories.

### Install the VSCode Extensions You’ll Need

Launch VSCode and install the following extensions (you can search for them in the Extensions tab on the left sidebar):

- **Dev Containers** – allows you to open and work inside Docker-based development environments  
- **Docker** – helps visualize and manage containers  
- **WSL** – (Windows only) lets VSCode connect to your WSL Ubuntu environment  

You can install them all at once by running this command in a terminal:

```bash
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-vscode-remote.remote-wsl
code --install-extension ms-azuretools.vscode-docker
```

---

## Setup - Windows

This project is developed and built inside Docker containers running on **WSL2 (Windows Subsystem for Linux)**. We will use **Ubuntu (latest)** as the default WSL distribution to keep the setup consistent across all contributors.

*In this guide, **WSL2** and **WSL** are used interchangeably; however, they both mean the same thing.*

The development workflow is:

1. **Run Docker alongside WSL2 (via Docker Desktop).**
2. **Build and run code inside of the development containers using VSCode.**
3. **Use Git inside WSL2** for source control.

### Install WSL2 and Ubuntu

For reference, here's Microsoft’s official guide: [Install WSL](https://learn.microsoft.com/en-us/windows/wsl/install).

To install WSL, open any terminal in windows and enter the following:

```bash
wsl --install
```

**TIME TO REBOOT!** Changes will not be made until the system is rebooted.

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

To run WSL, **press the windows key and search WSL or Ubuntu** and launch the executable. Then run the following command:

```bash
sudo apt update && sudo apt upgrade -y
```

It will ask you to create a **username** - I always do `abotticchio` or `[firstInitial+lastname]`. It will also prompt you to create a password - **keep this light and short!** Keep in mind, when typing in your password in a terminal, *your characters won't show on screen* but will still be valid (this is a canon experience for first time linux users).

### Install Git in WSL

You want git available in your Ubuntu shell so you can clone our AEAC2026 repo. Enter the following in WSL:

```bash
sudo apt update && sudo apt install git -y
```

Verify by entering the following:

```bash
git --version # You should see output from this command
```

### Make an SSH Key

Now that Git is installed, let’s create an SSH key so you can securely authenticate with GitHub without typing your password every time.

First, generate a new SSH key pair:

```bash
ssh-keygen
```

When prompted to **"Enter a file in which to save the key",** just press Enter (3 times) to accept the default path:

```bash
~/.ssh/id_ed25519
```

Now, let’s copy your **public key** so you can add it to GitHub. Run:

```bash
cat ~/.ssh/id_ed25519.pub
```

This will print a line starting with `ssh-ed25519`. **Copy that entire line.**

Then:

1. Go to Go to your GitHub profile &rarr; **Settings** &rarr; **SSH and GPG Keys** &rarr; **New SSH Key**
2. Give it a recognizable title (e.g. “WSL Laptop”)
3. Paste your key into the **Key** field
4. Click **Add SSH key**

Finally, test your connection to GitHub:

```bash
ssh -T git@github.com
```

You should see:

```bash
Hi [username]! You've successfully authenticated, but GitHub does not provide shell access.
```

If you see this, then you're good to go!

### Clone the Repository

Inside WSL, make a **`git`** directory in your **home (`~`)** directory. This is where we will clone AEAC2026 and other git repositories into. First navigate to your home directory with `cd` or `cd ~` (using `cd` allows you to navigate directories, i.e `cd ~`), and make a `git` directory using `mkdir git` (using `mkdir` allows you to make directories, i.e `mkdir git`).

Then navigate into your git directory (`cd ~/git`) and clone the repo using SSH with `git clone <ssh_url>`. Your commands should look like this bellow:

```bash
cd ~
mkdir git
cd git
git clone git@github.com:Queen-s-Aerospace-Design-Team/AEAC2026.git
```

Enter `Yes` if you are cloning with SSH for the first time (i.e. just after making an SSH key).

### Install Docker Desktop

Download and install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/). Docker desktop is a windows/mac application that will automatically detect running WSL instances and you'll be able to access **`docker`** commands within WSL without needing to install it explicitly.

During setup:
- Enable **WSL2 backend**.
- Integrate Docker Desktop with your **Ubuntu** WSL2 distribution.

After installation ensure that docker desktop is running on Windows. Then open a terminal in WSL and confirm Docker works:

```bash
docker --version
```

If this doesn't work, try restarting your environment by first quitting docker desktop, restarting WSL (press Cntrl + D in the WSL terminal to exit it), and try again.

### Open the AEAC2026 repo in VSCode

Now that Docker, VSCode, and Git are installed. We can now start pulling the docker container using the VSCode Dev Containers extension. We do this by opening the AEAC2026 repo in VSCode.

Personally, I like to do this using the terminal:

```bash
cd ~/git/AEAC2026
code .  # 'code' means VSCode, and we supply '.' as the current directory 
        # for the location we want to open VSCode into
```

We have a folder in the AEAC2026 repo called **`.devcontainer`**. This folder holds all the information required to create our development environment. Including the docker image (which is stored as a package on our GitHub organization), environment variables (defined in the `compose.yml` files), and other VSCode specific settings defined in the `.devcontainer/devcontainer.json` configuration file.

When opening VSCode, you should see a prompt asking to **Reopen in Container**, select yes. If you do not, you can trigger the same action by opening VSCode's command palette with `Cntrl/Command + Shift + P` and typing `>Dev Containers: Rebuild and Reopen in Container` then selecting that option.

After accepting the prompt the docker image will begin downloading. The entire process of opening the container should take 10 $-$ 15 minutes.

### Troubleshooting

- Pressing `Cntrl + D` in a WSL terminal will force a shutdown of WSL.
- Run `wsl --shutdown` in a **Windows terminal** and reopen Ubuntu.'
- Restart Docker by ending the process in the **tray** of your taskbar.

---

## Setup - MacOS

This project is developed and built inside Docker containers running on **MacOS** with the help of VSCode's dev containers extension. We currently do not use an other VMs or containers other than what is developed in-house and used inside of VSCode's dev containers extension.

The development workflow is:

1. **Run Docker Desktop.**
2. **Open development container inside of VSCode.**
3. **Build and run code inside of said development container**
4. **Use Git on MacOS** for source control.

### Install Homebrew

For macOS, we’ll use **Homebrew**, the package manager for Mac, to install Git and other development tools.

To install Homebrew, open your **Terminal** (press `Cmd + Space`, type “Terminal”, and hit Enter) and run the following command:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Once installed, add Homebrew to your shell's PATH. For example:

```bash
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
```

You can verify Homebrew is installed correctly with: 

```bash
brew --version
```

### Install Git with Homebrew

You want git available in your MacOS terminal so you can clone our AEAC2026 repo. Now that Homebrew is installed, user it to install Git:

```bash
brew install git
```

Verify by entering the following:

```bash
git --version # You should see output from this command
```

### Make an SSH Key

Now that Git is installed, let’s create an SSH key so you can securely authenticate with GitHub without typing your password every time.

First, generate a new SSH key pair:

```bash
ssh-keygen
```

When prompted to **"Enter a file in which to save the key",** just press Enter (3 times) to accept the default path:

```bash
~/.ssh/id_ed25519
```

Now, let’s copy your **public key** so you can add it to GitHub. Run:

```bash
cat ~/.ssh/id_ed25519.pub
```

This will print a line starting with `ssh-ed25519`. **Copy that entire line.**

Then:

1. Go to Go to your GitHub profile &rarr; **Settings** &rarr; **SSH and GPG Keys** &rarr; **New SSH Key**
2. Give it a recognizable title (e.g. “WSL Laptop”)
3. Paste your key into the **Key** field
4. Click **Add SSH key**

Finally, test your connection to GitHub:

```bash
ssh -T git@github.com
```

You should see:

```bash
Hi [username]! You've successfully authenticated, but GitHub does not provide shell access.
```

If you see this, then you're good to go!

### Clone the Repository

Inside your **home directory** (`~`). This is where you'll keep the AEAC2026 repository and any others you clone in the future. 

First navigate to your home directory with `cd` or `cd ~` (using `cd` allows you to navigate directories, i.e `cd ~`), and make a `git` directory using `mkdir git` (using `mkdir` allows you to make directories, i.e `mkdir git`).

Then navigate into your git directory (`cd ~/git`) and clone the repo using SSH with `git clone <ssh_url>`. Your commands should look like this bellow:

```bash
cd ~
mkdir git
cd git
git clone git@github.com:Queen-s-Aerospace-Design-Team/AEAC2026.git
```

If you’re cloning via SSH for the first time, you’ll be asked to confirm the connection — type **Yes** when prompted.

### Install Docker Desktop

Download and install [Docker Desktop for MacOS](https://docs.docker.com/desktop/setup/install/mac-install/). Docker Desktop allows you to run containers locally and integrates directly with VSCode.

After installation: 

- Make sure Docker Desktop is running (you should see the whale icon in your menu bar)
- Open a new terminal and verify Docker works with:

```bash
docker --version
```

If Docker commands don’t work immediately, try quitting and reopening Docker Desktop or restarting your terminal.

### Open the AEAC2026 repo in VSCode

Now that Docker, VSCode, and Git are installed. We can now start pulling the docker container using the VSCode Dev Containers extension. We do this by opening the AEAC2026 repo in VSCode.

Personally, I like to do this using the terminal:

```bash
cd ~/git/AEAC2026
code .  # 'code' means VSCode, and we supply '.' as the current directory 
        # for the location we want to open VSCode into
```

We have a folder in the AEAC2026 repo called **`.devcontainer`**. This folder holds all the information required to create our development environment. Including the docker image (which is stored as a package on our GitHub organization), environment variables (defined in the `compose.yml` files), and other VSCode specific settings defined in the `.devcontainer/devcontainer.json` configuration file.

When opening VSCode, you should see a prompt asking to **Reopen in Container**, select yes. If you do not, you can trigger the same action by opening VSCode's command palette with `Cntrl/Command + Shift + P` and typing `>Dev Containers: Rebuild and Reopen in Container` then selecting that option.

After accepting the prompt the docker image will begin downloading. The entire process of opening the container should take 10 $-$ 15 minutes.

### Troubleshooting

- If Docker isn’t responding, quit and restart Docker Desktop.

---

## Setup - Native Linux

**TODO**

---

# Other

## Running ROS examples

Navigate to the `ros_examples_cpp` and enter the command:

```bash
colcon build && source install/setup.sh
```

To build all packages under the `AEAC2026/ros_examples_cpp/src` directory. Then run `ros2 run [pkg_name] [executable_name]`. For example, in one terminal instance run, `ros2 run pubsub_01 publisher`, and in another terminal instance run, `ros2 run pubsub_01 subscriber`.

Feel free to experiment with the ROS2 examples.

## (Legacy) Building Docker Images

***CAUTION:** Building a docker container can take a while. Try to complete this step in one sitting.*

Navigate to the project directory using the `cd` command (Ex: `cd ~/git/AEAC2026`).Then navigate to the docker image folder `docker-dev` and build the image:

```bash
docker build -t qadt-image .
```

The '.' (which always means your current working directory) specifies the directory in which the Dockerfile is located to build the image. In this case, if you navigated to the correct directory, `AEAC2026/docker-dev`. We use the `-t` argument, which corresponds to the image tag, to give the image a name `qadt-image`.

You can always list the docker images you have built using:

```
docker images
----------------
REPOSITORY   TAG       IMAGE ID       CREATED       SIZE
qadt-image   latest    cae451abaf44   12 days ago   17.6GB
```

Run the container using the `runContainer.sh` script found under `AEAC2026/docker-dev` by executing the following:

```bash
./runContainer.sh
```

The `./` at the beginning means to execute. If `runContainer.sh` existed in another directory, you would instead enter: `./path/to/script/runContainer.sh`.

If this does not work, enable permissions to execute the script with:

```bash
sudo chmod +x runContainer.sh
```

`chmod` changes the file mode bits of files or directories to set read, write, and execute permissions for the owner, group, and others. In this case, we used `+x` which to enables execution of the script.

Once the container is running, you'll notice that the user is set to `qadt@container`.

Username: `qadt`
Password: `aero`

You will need to enter the password `aero` when typing `sudo` commands in the container shell.

Feel free to explore the home directory of the dev container.

Once you want to exit the container, enter `exit`.

Confirm that no docker container is running by listing the running containers:

```bash
docker ps     # To see running containers
docker ps -a  # To see all containers
```

If you ever need to stop a container explicitly, run:

```bash
docker stop qadt-dev
```

Or stop it in the Docker Desktop GUI.

If you want to resume your container session, run:

```bash
docker start -i qadt-dev # '-i' means interactive and will attach to its terminal
```

---

# Software Executive Team (2025/26)

- **Max Dizy** $-$ *AEAC Co-Captain*
- **Kalena McCloskey** $-$ *Software Director*
- **Ethan Milburn** $-$ *Software Research (ICUAS) Manager*
- **Matthew Lones** $-$ *Computer Vision Manager*
- **Anthony Botticchio** $-$ *Autonomy Manager*