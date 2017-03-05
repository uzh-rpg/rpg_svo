// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_BUNDLE_ADJUSTMENT_H_
#define SVO_BUNDLE_ADJUSTMENT_H_

#include <svo/global.h>

namespace g2o {
class EdgeProjectXYZ2UV;
class SparseOptimizer;
class VertexSE3Expmap;
class VertexSBAPointXYZ;
}

namespace svo {

typedef g2o::EdgeProjectXYZ2UV g2oEdgeSE3;
typedef g2o::VertexSE3Expmap g2oFrameSE3;
typedef g2o::VertexSBAPointXYZ g2oPoint;

class Frame;
class Point;
class Feature;
class Map;

/// Local, global and 2-view bundle adjustment with g2o
namespace ba {

/// Temporary container to hold the g2o edge with reference to frame and point.
struct EdgeContainerSE3
{
  g2oEdgeSE3*     edge;
  Frame*          frame;
  Feature*        feature;
  bool            is_deleted;
  EdgeContainerSE3(g2oEdgeSE3* e, Frame* frame, Feature* feature) :
    edge(e), frame(frame), feature(feature), is_deleted(false)
  {}
};

/// Optimize two camera frames and their observed 3D points.
/// Is used after initialization.
void twoViewBA(Frame* frame1, Frame* frame2, double reproj_thresh, Map* map);

/// Local bundle adjustment.
/// Optimizes core_kfs and their observed map points while keeping the
/// neighbourhood fixed.
void localBA(
    Frame* center_kf,
    set<FramePtr>* core_kfs,
    Map* map,
    size_t& n_incorrect_edges_1,
    size_t& n_incorrect_edges_2,
    double& init_error,
    double& final_error);

/// Global bundle adjustment.
/// Optimizes the whole map. Is currently not used in SVO.
void globalBA(Map* map);

/// Initialize g2o with solver type, optimization strategy and camera model.
void setupG2o(g2o::SparseOptimizer * optimizer);

/// Run the optimization on the provided graph.
void runSparseBAOptimizer(
    g2o::SparseOptimizer* optimizer,
    unsigned int num_iter,
    double& init_error,
    double& final_error);

/// Create a g2o vertice from a keyframe object.
g2oFrameSE3* createG2oFrameSE3(
    Frame* kf,
    size_t id,
    bool fixed);

/// Creates a g2o vertice from a mappoint object.
g2oPoint* createG2oPoint(
    Vector3d pos,
    size_t id,
    bool fixed);

/// Creates a g2o edge between a g2o keyframe and mappoint vertice with the provided measurement.
g2oEdgeSE3* createG2oEdgeSE3(
    g2oFrameSE3* v_kf,
    g2oPoint* v_mp,
    const Vector2d& f_up,
    bool robust_kernel,
    double huber_width,
    double weight = 1);

} // namespace ba
} // namespace svo

#endif // SVO_BUNDLE_ADJUSTMENT_H_
