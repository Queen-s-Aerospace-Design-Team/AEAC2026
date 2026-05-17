# Objectives

TLDR; Matt will be at the competition and I (Anthony) will be absent. Matt will be focused on integrating his python work into ROS packages and I will focus on deployment and mission structure. The idea is so have two docker services available for launch `task-1` and `task-2`. Each with their respective services enabled (i.e. perception and no autonomy for `task-1` and google-uploader for `task-2`).

We are to try our best. It obviously isn't ideal that Matt and I could not get together to test and integrate ourselves, and that I am absent from the competition. Then again, the circumstance in which software is the last piece of work to verify is only when every other sub-team's work is integrated and functioning flawlessly - then software can execute as expected.

That is the natural consequence for a design team like this (and hopefully that can change next year).

## Matt

- Create google account for competition
    - Must be a service account for us to grab a API key from for google drive uploads
    - Since Matt will be at the competition, he will be able to debug incase 2FA is triggered
- Task 1 seems to have the majority of work done
- Integrate python files into a ROS package which can be built and launched via a docker service listed under `deployment/`. Currently, the `compose.perception.yml` does not execute `ros2 run [pkg_name] [node_name]` for a perception node. If we want a containerized environment, ROS is easy. Otherwise, we could just run the file/python package itself (whatever it may be).

## Anthony

- Deployment environment must be flawless
    - Everything from the docker images, `compose.*.yml` launch services, and the `deploy.sh` script under `scripts/` must work as intended and be easy to use
        - `deploy.sh` script should have easy-to-use instructions
        - `up` - services start up
        - `down` - shutdown and remove running containter instances
        - `restart` - restart running container instances
        - `logs` - attach to stdout
    - Enable CI/CD to rebuild `deployment-px4:latest` on every `AEAC2026` commit to main
        - This is partially done from what Harnake did earlier on a branch.
- Servo ROS node is done, but *needs* to be tested. It is sitting on task `#47` and in a branch. I have not PR'd it since I have not had a chance to test it. It was also purely vibe-coded.
- Watergun is an uncertain situation. I am imaging that it will be connected to **GPIO** and we just need to access the correct GPIO pin and send a signal that shoots the water.
    - This requires rigorous testing (yikes) as we need to get the right angle/pitch for it to even be considered autonomous.
- My focus is purely on deployment and practicality. Meaning that the autonomy portion of `task-2` should not be attempted until the aforemetnioned is completed. Or at least we are confident in it.