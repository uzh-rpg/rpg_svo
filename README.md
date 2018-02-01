Modify to svo2.0 to support stereo camera, with:
- use stereo initialzation instead of mono initialzation by SfM.
- extend the sparse image alignment, pose optimization to support multi camera joint optimization. For other steps of feature alignment and mapping which are independent of camera amount, only need add another loop to iterate process each camera data.
- depth filter use history frames to update new seeds, which suggested by the SVO2.0 paper.
- enhance the keyframe select strategy.

=======================================================================================

SVO
===

This code implements a semi-direct monocular visual odometry pipeline.

Video: http://youtu.be/2YnIMfw6bJY

Paper: http://rpg.ifi.uzh.ch/docs/ICRA14_Forster.pdf

#### Disclaimer

SVO has been tested under ROS Groovy, Hydro and Indigo with Ubuntu 12.04, 13.04 and 14.04. This is research code, any fitness for a particular purpose is disclaimed.


#### Licence

The source code is released under a GPLv3 licence. A closed-source professional edition is available for commercial purposes. In this case, please contact the authors for further info.


#### Citing

If you use SVO in an academic context, please cite the following publication:

    @inproceedings{Forster2014ICRA,
      author = {Forster, Christian and Pizzoli, Matia and Scaramuzza, Davide},
      title = {{SVO}: Fast Semi-Direct Monocular Visual Odometry},
      booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
      year = {2014}
    }
    
    
#### Documentation

The API is documented here: http://uzh-rpg.github.io/rpg_svo/doc/

#### Instructions

See the Wiki for more instructions. https://github.com/uzh-rpg/rpg_svo/wiki

#### Contributing

You are very welcome to contribute to SVO by opening a pull request via Github.
I try to follow the ROS C++ style guide http://wiki.ros.org/CppStyleGuide
