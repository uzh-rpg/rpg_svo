## SVO Analysis

#### Download Test Data

Create a folder where you will store all datasets, e.g.:

    mkdir Datasets

In your `.bashrc` script, create a new environment variable pointing to the dataset folder:

    export SVO_DATASET_DIR=${HOME}/Datasets

Source your new `.bashrc` script, go to the new dataset folder and download the test data:

    source `~/.bashrc`
    cd ${SVO_DATASET_DIR}
    wget http://rpg.ifi.uzh.ch/datasets/flying_room_1_rig_1.tar.gz -O - | tar -xz


#### Run Benchmark

In SVO you need to set the `TRACE` flag to `TRUE` and recompile SVO.
Afterwards run

    rosrun svo_analysis benchmark.py flying_room_1_fast

The script will create a new folder in `svo_analysis/results/`, run svo on the dataset and save the tracefile in this folder and at the end it will run some scripts to generate plots from the tracefile.