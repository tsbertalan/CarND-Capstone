This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

* Either clone this repository to /capstone, or clone it elsewhere and symlink it there with `ln -s "$CLONEDIR" /capstone`.

* Make a python 2.7 virtual environment. [*virtualenvwrapper*](https://virtualenvwrapper.readthedocs.io/en/latest/) is great for this.

* If installing on a machine with a CUDA- and TensorFlow-capable GPU:
  * Follow [TensorFlow's](https://www.tensorflow.org/install/install_linux#tensorflow_gpu_support) installation instructions, including the link to instructions for installing the CUDA toolkit and cuDNN. You may want to insert any commands for modifying your environment variables, such as `LD_LIBRARY_PATH`, into your ~/.profile file.
  * Change `tensorflow` to `tensorflow-gpu` in requirements.txt.
  * Note that CUDA 8 may be necessary for running with Python 2.7, as is the norm for ROS. This can be installed alongside CUDA 9 with `sudo apt-get install cuda-8-0`.

* Reproduce the installation steps from Dockerfile, including running tod_coco_install.sh. Note that you'll first need to run e.g. `export ROS_DISTRO=kinetic` depending on your ROS distribution. Note that tod_coco_install.sh modifies your .bashrc file.

* Still in the virtualenv, run `pip uninstall em` and `pip install` [`catkin_pkg`](https://stackoverflow.com/questions/43024337/why-this-error-when-i-try-to-create-workspaces-in-ros) [`em`](https://answers.ros.org/question/257331/python-module-empy-missing-tutorials/) [`rospkg`](https://answers.ros.org/question/39657/importerror-no-module-named-rospkg/).

* In /capstone/ros, run `catkin_make`.

* Either in a terminal where `roslaunch` is to be run, or permanently via ~/.bashrc or ~/.profile, run `source /capstone/ros/devel/setup.sh`.

* Download [luu_real.pb](https://www.dropbox.com/s/51db8bato54plx1/luu_real.pb?dl=1) and [luu_sim.pb](https://www.dropbox.com/s/jbzy1skrsso15bf/luu_sim.pb?dl=1) and put them both in /capstone/ros/src/tl_detector/light_classification/saved_nets, making this directory if necessary.

* Optionally, install the the Rviz plugins with `sudo apt-get install ros-kinetic-jsk-rviz-plugins libbullet-dev libsdl-image1.2-dev libsdl-dev`.



### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

In /capstone/ros, after establishing the right `$PYTHONPATH` environment variable to (likely via ~/.profile) and sourcing /capstone/ros/devel/setup.sh, run the controller and light-detector with `roslaunch launch/styx.launch`.

When the lauch has completed and all nodes are running, use run the simulator, and uncheck "Manual", and check "Camera".



### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
