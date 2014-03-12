Download Synthetic Test Data:
-----------------------------

Downloads and extracts the test data in the folder `data`:

    cd rpg_slam/svo/test/data
    wget http://rpg.ifi.uzh.ch/datasets/flying_room_1_rig_1.tar.gz -O - | tar -xz

If you want to store the data in another location (for instance, if you maintain a special folder with all datasets), then you can move the dataset there and set the environment variable `TEST_DATA_DIR` in your `.bashrc` like this:

    export SVO_DATASET_DIR=${HOME}/Datasets
