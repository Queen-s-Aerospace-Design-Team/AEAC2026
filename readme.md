# QADT 2025/26 Software Repository

The purpose of this repository is to house all software associated with development environments, containers, computer vision, autonomy, and more.

## Docker container setup - Dual Boot

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

## Running ROS examples

With ROS2 installed on your system, navigate to `ros_examples_cpp` and enter the command:

```bash
colcon build && source install/setup.sh
```

To build all packages under the `QADT2026/ros_examples_cpp/src` directory. Then run `ros2 run [pkg_name] [executable_name]`. For example, in one terminal instance run, `ros2 run service_02 server`, and in another terminal instance run, `ros2 run service_02 client`.

Feel free to experiment with the ROS2 examples.

# Docker Container Setup - WSL

Docker Desktop has native integration with WSL2. Meaning that the docker engine will attach to your WSL instance and you'll be able to access `docker` commands without having to install it in WSL.

Ensure that you have WSL installed first, then go ahead and install [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/). Make sure to select the 



# Software Execs

- **Max Dizy** $-$ *AEAC Co-Captain*
- **Kalena McCloskey** $-$ *Software Director*
- **Ethan Milburn** $-$ *Software Research (ICUAS) Manager*
- **Matthew Lones** $-$ *Computer Vision Manager*
- **Anthony Botticchio** $-$ *Autonomy Manager*
