SVO
===

This code implements a semi-direct visual odometry pipeline that is described in the paper 

*   C. Forster, M. Pizzoli, D. Scaramuzza,
    "__SVO: Fast Semi-Direct Monocular Visual Odometry__,"
    IEEE International Conference on Robotics and Automation (ICRA), 2014.

Video: http://youtu.be/2YnIMfw6bJY

Disclaimer
----------

SVO has been tested under ROS Groovy and Hydro and Ubuntu 12.04 and 13.04. This is research code, any fitness for a particular purpose is disclaimed.


Licence
-------

The source code is released under a GPLv3 licence. A professional edition license for closed-source projects is also available. Please contact `forster at ifi dot uzh dot ch` for further information.

Documentation
-------------

The API is documented here: http://uzh-rpg.github.io/rpg_svo/doc/

Installation Instructions
-------------------------

SVO can be used conveniently with ROS (www.ros.org).
We use two workspaces, one for the plain CMake projects `Sophus`, `Fast` and optionally `g2o` and another workspace for the ROS-Catkin projects `rpg_vikit` and `rpg_svo`. Make sure to clone in the right folder.

#### Sophus - Lie groups

Sophus by Hauke Strasdat implements Lie groups that we need to describe rigid body transformations. Checkout in your workspace for plain CMake projects.

    cd workspace
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus
    git checkout a621ff
    mkdir build
    cd build
    cmake ..
    make

You don't need to install the library since `cmake ..` writes the package location to `~.cmake/packages/` where CMake can later find it.

#### Fast - Corner Detector

The Fast detector by Edward Rosten is used to detect corners.
To simplify installation we provide a CMake package that contains the fast detector from the libCVD library (http://www.edwardrosten.com/cvd/).

    cd workspace
    git clone https://github.com/uzh-rpg/fast.git
    cd fast
    mkdir build
    cd build
    cmake ..
    make

#### g2o - General Graph Optimization _OPTIONAL_

Only required if you want to run bundle adjustment. It is not necessary for visual odometry. In fact, we don't run it on our MAVs.
g2o requires the following system dependencies: `cmake, libeigen3-dev, libsuitesparse-dev, libqt4-dev, qt4-qmake, libqglviewer-qt4-dev`, install them with `apt-get`
    
I suggest an out-of-source build of g2o:

    cd workspace
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

If you don't want to make a system install, then you can replace the cmake command with `cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/installdir` 

#### vikit - Some useful tools that we need

vikit for contains camera models, some math and interpolation functions that SVO needs.
vikit is a catkin project, therefore, download it into your catkin workspace source folder.

    cd catkin_ws/src
    git clone https://github.com/uzh-rpg/rpg_vikit.git

#### ROS dependencies

Sometimes `cmake-modules` is missing (required for including Eigen in ROS Indigo):

    sudo apt-get install ros-hydro-cmake-modules

(replace `hydro` with your distribution).

#### SVO

Now we are ready to build SVO.
Clone it into your catkin workspace

    cd catkin_ws/src
    git clone https://github.com/uzh-rpg/rpg_svo.git

If you installed g2o then set `HAVE_G2O` in `svo/CMakeLists.txt` to TRUE.

Finally, build:

    catkin_make
    

Run SVO on a Dataset
-------------------------

Download this example dataset: [rpg.ifi.uzh.ch/datasets/airground_rig_s3_2013-03-18_21-38-48.bag](http://rpg.ifi.uzh.ch/datasets/airground_rig_s3_2013-03-18_21-38-48.bag)

Open a new console and start SVO with the prepared launchfile:

    roslaunch svo_ros test_rig3.launch
    
Now you are ready to start the rosbag. Open a new console and change to the directory where you have downloaded the example dataset. Then type:

    rosbag play airground_rig_s3_2013-03-18_21-38-48.bag
    
Now you should see the video with tracked features (green) and in RViz how the camera moves.
If you want to see the number of tracked features, fps and tracking quality, run the GUI.

SVO GUI
-------

Type `rosrun rqt_svo rqt_svo` to run the SVO widget that displays the number of tracked features, the frame rate and provides some interface buttons.

If the widget is not found, try this:

    rm ~/.config/ros.org/rqt_gui.ini
    rosrun rqt_svo rqt_svo


Keyboard Shortcuts
------------------

Make sure to active the console window when pressing the keys.

* `s`   Start/Restart
* `q`   Quit
* `r`   Reset

Camera Calibration
------------------

You need a calibrated camera to run SVO. We use the Matrix Vision Bluefox cameras in our lab, which have VGA resolution and global shutter.
SVO supports three camera models: 

1. The `ATAN` model - our preference - which is also used by _PTAM_. This model uses the _FOV_ distortion model of "Deverneay and Faugeras, Straight lines have to be straight, 2001". You can calibrate your camera with this model by using the calibration tool in this package: https://github.com/ethz-asl/ethzasl_ptam We prefer this model because the projection and unprojection can be computed faster than with the other models. Further, the industral cameras that we use have neglectable tangential distortion.
2. The `Pinhole` model with three radial and two tangential distortion parameters. This model is standard in OpenCV and ROS. You can use the ROS camera calibration tool: http://wiki.ros.org/camera_calibration
3. The `Ocam` model by Davide Scaramuzza which can be used to model cameras with high field of view or even omnidirectional cameras. Use the OCamCalib toolbox to calibrate your camera: http://rpg.ifi.uzh.ch/software_datasets.html
Although SVO can be used with this camera model, the algorithm does not work yet with omnidirectional cameras.

We save the calibration in _YAML_ format (see examples in `svo_ros/param`) and include it in the launchfiles (see examples in `svo_ros/launch`).

Parameter Settings
------------------

A description of all parameters which can be set via the launchfile is provided in `svo/include/config.h`. The default parameters can be viewed in `svo/src/config.cpp`. Moreover, some additional parameters (mainly rostopic names etc.) are read from the ros parameter server in `svo_ros/vo_node.cpp`.

#### Forward motions

The current keyframe selection criterion is designed for downward looking cameras. This is one reason why SVO does not work well for forward motions (e.g., to be used on a car).

#### Image resolution

The current parameters are tuned for WVGA resolution. If you use a higher resolution camera, then the pyramid levels should be increased as well as the reprojection error thresholds.


Contributing
------------

You are very welcome to contribute to SVO by opening a pull request via Github.
I try to follow the ROS C++ style guide http://wiki.ros.org/CppStyleGuide

Citing
------

If you use SVO in an academic context, please cite the following publication:

    @inproceedings{Forster2014ICRA,
      author = {Forster, Christian and Pizzoli, Matia and Scaramuzza, Davide},
      title = {{SVO}: Fast Semi-Direct Monocular Visual Odometry},
      booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
      year = {2014}
    }
