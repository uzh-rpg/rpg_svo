#### Download Test Data

Create a folder where you will store all datasets, e.g.:

    mkdir Datasets

In your `.bashrc` script, create a new environment variable pointing to the dataset folder:

    export SVO_DATASET_DIR=${HOME}/Datasets

Source your new `.bashrc` script, go to the new dataset folder and download the test data:

    source ~/.bashrc
    cd ${SVO_DATASET_DIR}
    wget http://rpg.ifi.uzh.ch/datasets/sin2_tex2_h1_v8_d.tar.gz -O - | tar -xz

You can then run the tests, e.g.:

    rosrun svo test_pipeline

or if you are not using ROS:

    cd svo/bin
    ./test_pipeline
