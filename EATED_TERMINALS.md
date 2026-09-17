# EATED Terminator launcher

Run the following command from a graphical terminal:

```bash
eated
```

It opens an independent Terminator window with nine panes. Each pane sources
ROS 2 Humble and `/home/fcsl/eated_ws/install/setup.bash`, then waits with its
command prefilled. Commands are not started until Enter is pressed.

Start panes in numerical order. In particular, wait for hardware controllers
before pane 2, wait for camera topics before pane 4, and clear the robot's
workspace before starting pane 7 because the arm moves HOME on startup.
