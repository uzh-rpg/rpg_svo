## SVO Analysis

#### Download Test Data

Create a folder where you will store all datasets, e.g.:

    mkdir Datasets

In your `.bashrc` script, create a new environment variable pointing to the dataset folder:

    export SVO_DATASET_DIR=${HOME}/Datasets

Source your new `.bashrc` script, go to the new dataset folder and download the test data:

    source ~/.bashrc
    cd ${SVO_DATASET_DIR}
    wget http://rpg.ifi.uzh.ch/datasets/sin2_tex2_h1_v8_d.tar.gz -O - | tar -xz

#### Run Benchmark

In the _CMakeLists.txt_ of SVO you need to set the `TRACE` flag to `TRUE` and recompile SVO.
Afterwards run

    rosrun svo_analysis benchmark.py sin2_tex2_h1_v8_d

The script will create a new folder in `svo_analysis/results/`, run svo on the dataset and save the tracefile in this folder and at the end it will run some scripts to generate plots from the tracefile.
  
