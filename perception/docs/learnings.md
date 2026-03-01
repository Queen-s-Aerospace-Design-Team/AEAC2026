**Two important daemons:**

1. nvargus_deamon:

NVIDIA's camera service layer which sits between applications and physical camera hardware (nvidia capture card). Every time you open a ZED X camera, our app talks to nvargus-daemon, which manages the actual sensor capture, ISP (image signal processing), and frame delivery. Runs by default on JetPack.

`sudo systemctl status nvargus-daemon`

Things to watch out for: If it crashes or gets into a bad state, you get those Argus timeout errors. The fix is always restarting it. It can also get stuck if a previous app (like ZED Explorer) didn't close the camera cleanly, so always make sure you quit ZED Explorer before running your Python scripts, and vice versa (two processes cant share the camera). 

`sudo systemctl restart nvargus-daemon`

2. zed_x_daemon:

This is StereoLabs' own daemon, specifically for ZED X series cameras (which is what you have). It handles hardware-level features specific to the ZED X — things like the hardware encoder (H264/H265), sensor synchronization, and firmware communication.

`sudo systemctl status zed_x_daemon` / `sudo systemctl restart zed_x_daemon`
