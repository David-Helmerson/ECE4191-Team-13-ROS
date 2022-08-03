# ECE4191-Team-13-ROS
ROS2 Foxy packages for the robot. Instructions here are in reference to linux installations of ROS unless specified otherwise.\
My hope is that it won't be necessary for anyone else to need to write/build ROS packages, so most instructions are kept to a minimum.  

---

## Installing ROS2 Foxy
Download the appropirate packages from the [installation guide page](https://docs.ros.org/en/foxy/Installation.html). Since the Pi is running Ubuntu 20.04 it may be safest to install on a similar environment, though if you are able to setup on a different OS it shouldn't be too much of a problem.

Depending on if you are using ROS often, it may be helpful to automatically source your ROS installation on startup. This can be done via:  
`echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc`.

---

## Installing colcon
Colcon is a tool used for building packages in ROS. It can be installed via `sudo apt install python3-colcon-common-extensions`.\
A couple other useful(?) shortcuts for navigating between ROS packages, as well as tab completion can be created with:\
`echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc` \
`echo "export _colcon_cd_root=/opt/ros/foxy/" >> ~/.bashrc` \
`echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc`

For extra detail on using colcon as well as instuctions for other operating systems, see [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

---

## Creating the ROS workspace and cloning this repo
The ROS workspace is where all of the code for the Pi will be held. To create a workspace on your system, first create a directory with an appropriate name (e.g. dev_ws in the examples) with a subdirectory named src. On linux this can be done with `mkdir -p ~/dev_ws/src`.

Navigate into the src folder (`cd ~/dev_ws/src`) and clone this repository, you should see the repository folder containing example_package within the the src folder. Next, navigate to the root of the workspace (the dev_ws folder) and run `colcon build`. This should take a few seconds as example_packages starts and finishes building. Once complete you should also see new folders within the workspace named build, install, and log.

Once a package withing a workspace has been built, the workspace must be sourced again. To do this, run the newly created setup script with `install/setup.bash`. Once this is done, you should be able to run the node within example_package with `ros2 run example_package example_node` and see a response.

For more detailed tutorials on creating a workspace and using colcon, see [the "Creating a Workspace" tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and the colcon tutorial in the previous part.

---

## File structure and workflow
Take a look at example_package (either in the command line or with a file navigator). Within this folder (package) there should be a subdirectory with the same name. This is where all of the code of both the ROS nodes for this package, and ***any custom modules they import*** is held.

When writing software for the robot, I imagine work will be divied up based on task (e.g. with a kanban). This way, anybody working on software for the robot who is not familiar with ROS can do so by writing a python module for an appropriate package, which can then be pushed to github and integrated into ROS soon after. Modules written in this way should still be easy to debug and test by simply running them individually from either the command line or your prefered IDE.

As for git workflow, this is still a work in progress but I'm expecting to have every incomplete package exist on a separate branch. Regardless, if you are unsure about what branch you should be working in please ask in slack or create your own (don't work off of main!). Also if you need help with git please ask, it took me an embarassingly long time to get somewhat comfortable with it since I was just too afraid to do so for the longest time.

---

## Afterword
This should get you all setup to be able to write software for the robot independant of your ROS knowledge. The ROS2 documentation contains [useful tutorials](https://docs.ros.org/en/foxy/Tutorials.html) if you want to get an understanding of how to use ROS yourself, the most important of which I've found to be:
- Understanding Nodes
- Understanding Topics
- Understanding Services (although I haven't actually had to make use of this myself)
- Understanding Parameters
- Launching Nodes
- Using colcon to build packages
- Creating a workspace
- Creating a package
- Writing a simple publisher and subscriber (I use this as a template a lot)
- Using parameters in a class
- Managing dependencies with rosdep (this should be introduced earlier than it is)
