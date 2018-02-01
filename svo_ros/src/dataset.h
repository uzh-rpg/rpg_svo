#ifndef __DATASET_H__
#define __DATASET_H__

#include <string>

#define DATASET1

// sf outside 1A 1B 1C, forward
#ifdef DATASET1
std::string video_file = "/home/symao/data/mynteye/20171107vins_outside/1/img.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171107vins_outside/1/imgts.txt";
#endif

// sf inside 1C4F, forward
#ifdef DATASET2
std::string video_file = "/home/symao/data/mynteye/20171107vins/img.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171107vins/imgts.txt";
#endif

// shiyan outside, downward
#ifdef DATASET3
std::string video_file = "/home/symao/data/mynteye/20171220/15/img_cut.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171220/15/img_cutts.txt";
#endif

// shiyan outside police station, downward
#ifdef DATASET4
std::string video_file = "/home/symao/data/mynteye/20171220/17/img_cut.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171220/17/img_cutts.txt";
#endif

// shiyan outside walk, downward
#ifdef DATASET5
std::string video_file = "/home/symao/data/mynteye/20171220/17/img_cut1.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171220/17/img_cut1ts.txt";
#endif

// sf outside 1A 1B 1C, forward, in TX2
#ifdef DATASET6
std::string video_file = "/home/nvidia/data/mynteye/outside/img.avi";
std::string video_ts_file = "/home/nvidia/data/mynteye/outside/imgts.txt";
#endif

#endif//__DATASET_H__