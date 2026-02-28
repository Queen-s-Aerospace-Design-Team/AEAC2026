**Two important daemons:**

1. nvargus_deamon: NVIDIA's camera service layer which sits between applications and physical camera hardware (nvidia capture card). Every time you open a ZED X camera, our app talks to nvargus-daemon, which manages the actual sensor capture, ISP (image signal processing), and frame delivery. Runs by default on JetPack.

`sudo systemctl status nvargus-daemon`