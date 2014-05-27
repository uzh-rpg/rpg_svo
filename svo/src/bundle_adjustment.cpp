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

#include <vikit/math_utils.h>
#include <boost/thread.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <svo/bundle_adjustment.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <svo/map.h>

#define SCHUR_TRICK 1

namespace svo {
namespace ba {

void twoViewBA(
    Frame* frame1,
    Frame* frame2,
    double reproj_thresh,
    Map* map)
{
  // scale reprojection threshold in pixels to unit plane
  reproj_thresh /= frame1->cam_->errorMultiplier2();

  // init g2o
  g2o::SparseOptimizer optimizer;
  setupG2o(&optimizer);

  list<EdgeContainerSE3> edges;
  size_t v_id = 0;

  // New Keyframe Vertex 1: This Keyframe is set to fixed!
  g2oFrameSE3* v_frame1 = createG2oFrameSE3(frame1, v_id++, true);
  optimizer.addVertex(v_frame1);

  // New Keyframe Vertex 2
  g2oFrameSE3* v_frame2 = createG2oFrameSE3(frame2, v_id++, false);
  optimizer.addVertex(v_frame2);

  // Create Point Vertices
  for(Features::iterator it_ftr=frame1->fts_.begin(); it_ftr!=frame1->fts_.end(); ++it_ftr)
  {
    Point* pt = (*it_ftr)->point;
    if(pt == NULL)
      continue;
    g2oPoint* v_pt = createG2oPoint(pt->pos_, v_id++, false);
    optimizer.addVertex(v_pt);
    pt->v_pt_ = v_pt;
    g2oEdgeSE3* e = createG2oEdgeSE3(v_frame1, v_pt, vk::project2d((*it_ftr)->f), true, reproj_thresh*Config::lobaRobustHuberWidth());
    optimizer.addEdge(e);
    edges.push_back(EdgeContainerSE3(e, frame1, *it_ftr)); // TODO feature now links to frame, so we can simplify edge container!

    // find at which index the second frame observes the point
    Feature* ftr_frame2 = pt->findFrameRef(frame2);
    e = createG2oEdgeSE3(v_frame2, v_pt, vk::project2d(ftr_frame2->f), true, reproj_thresh*Config::lobaRobustHuberWidth());
    optimizer.addEdge(e);
    edges.push_back(EdgeContainerSE3(e, frame2, ftr_frame2));
  }

  // Optimization
  double init_error, final_error;
  runSparseBAOptimizer(&optimizer, Config::lobaNumIter(), init_error, final_error);
  printf("2-View BA: Error before/after = %f / %f\n", init_error, final_error);

  // Update Keyframe Positions
  frame1->T_f_w_.rotation_matrix() = v_frame1->estimate().rotation().toRotationMatrix();
  frame1->T_f_w_.translation() = v_frame1->estimate().translation();
  frame2->T_f_w_.rotation_matrix() = v_frame2->estimate().rotation().toRotationMatrix();
  frame2->T_f_w_.translation() = v_frame2->estimate().translation();

  // Update Mappoint Positions
  for(Features::iterator it=frame1->fts_.begin(); it!=frame1->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
     continue;
    (*it)->point->pos_ = (*it)->point->v_pt_->estimate();
    (*it)->point->v_pt_ = NULL;
  }

  // Find Mappoints with too large reprojection error
  const double reproj_thresh_squared = reproj_thresh*reproj_thresh;
  size_t n_incorrect_edges = 0;
  for(list<EdgeContainerSE3>::iterator it_e = edges.begin(); it_e != edges.end(); ++it_e)
    if(it_e->edge->chi2() > reproj_thresh_squared)
    {
      if(it_e->feature->point != NULL)
      {
        map->safeDeletePoint(it_e->feature->point);
        it_e->feature->point = NULL;
      }
      ++n_incorrect_edges;
    }

  printf("2-View BA: Wrong edges =  %zu\n", n_incorrect_edges);
}

void localBA(
    Frame* center_kf,
    set<FramePtr>* core_kfs,
    Map* map,
    size_t& n_incorrect_edges_1,
    size_t& n_incorrect_edges_2,
    double& init_error,
    double& final_error)
{

  // init g2o
  g2o::SparseOptimizer optimizer;
  setupG2o(&optimizer);

  list<EdgeContainerSE3> edges;
  set<Point*> mps;
  list<Frame*> neib_kfs;
  size_t v_id = 0;
  size_t n_mps = 0;
  size_t n_fix_kfs = 0;
  size_t n_var_kfs = 1;
  size_t n_edges = 0;
  n_incorrect_edges_1 = 0;
  n_incorrect_edges_2 = 0;

  // Add all core keyframes
  for(set<FramePtr>::iterator it_kf = core_kfs->begin(); it_kf != core_kfs->end(); ++it_kf)
  {
    g2oFrameSE3* v_kf = createG2oFrameSE3(it_kf->get(), v_id++, false);
    (*it_kf)->v_kf_ = v_kf;
    ++n_var_kfs;
    assert(optimizer.addVertex(v_kf));

    // all points that the core keyframes observe are also optimized:
    for(Features::iterator it_pt=(*it_kf)->fts_.begin(); it_pt!=(*it_kf)->fts_.end(); ++it_pt)
      if((*it_pt)->point != NULL)
        mps.insert((*it_pt)->point);
  }

  // Now go throug all the points and add a measurement. Add a fixed neighbour
  // Keyframe if it is not in the set of core kfs
  double reproj_thresh_2 = Config::lobaThresh() / center_kf->cam_->errorMultiplier2();
  double reproj_thresh_1 = Config::poseOptimThresh() / center_kf->cam_->errorMultiplier2();
  double reproj_thresh_1_squared = reproj_thresh_1*reproj_thresh_1;
  for(set<Point*>::iterator it_pt = mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    // Create point vertex
    g2oPoint* v_pt = createG2oPoint((*it_pt)->pos_, v_id++, false);
    (*it_pt)->v_pt_ = v_pt;
    assert(optimizer.addVertex(v_pt));
    ++n_mps;

    // Add edges
    list<Feature*>::iterator it_obs=(*it_pt)->obs_.begin();
    while(it_obs!=(*it_pt)->obs_.end())
    {
      Vector2d error = vk::project2d((*it_obs)->f) - vk::project2d((*it_obs)->frame->w2f((*it_pt)->pos_));

      if((*it_obs)->frame->v_kf_ == NULL)
      {
        // frame does not have a vertex yet -> it belongs to the neib kfs and
        // is fixed. create one:
        g2oFrameSE3* v_kf = createG2oFrameSE3((*it_obs)->frame, v_id++, true);
        (*it_obs)->frame->v_kf_ = v_kf;
        ++n_fix_kfs;
        assert(optimizer.addVertex(v_kf));
        neib_kfs.push_back((*it_obs)->frame);
      }

      // create edge
      g2oEdgeSE3* e = createG2oEdgeSE3((*it_obs)->frame->v_kf_, v_pt,
                                       vk::project2d((*it_obs)->f),
                                       true,
                                       reproj_thresh_2*Config::lobaRobustHuberWidth(),
                                       1.0 / (1<<(*it_obs)->level));
      assert(optimizer.addEdge(e));
      edges.push_back(EdgeContainerSE3(e, (*it_obs)->frame, *it_obs));
      ++n_edges;
      ++it_obs;
    }
  }

  // structure only
  g2o::StructureOnlySolver<3> structure_only_ba;
  g2o::OptimizableGraph::VertexContainer points;
  for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
  {
    g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
      if (v->dimension() == 3 && v->edges().size() >= 2)
        points.push_back(v);
  }
  structure_only_ba.calc(points, 10);

  // Optimization
  if(Config::lobaNumIter() > 0)
    runSparseBAOptimizer(&optimizer, Config::lobaNumIter(), init_error, final_error);

  // Update Keyframes
  for(set<FramePtr>::iterator it = core_kfs->begin(); it != core_kfs->end(); ++it)
  {
    (*it)->T_f_w_ = SE3( (*it)->v_kf_->estimate().rotation(),
                         (*it)->v_kf_->estimate().translation());
    (*it)->v_kf_ = NULL;
  }

  for(list<Frame*>::iterator it = neib_kfs.begin(); it != neib_kfs.end(); ++it)
    (*it)->v_kf_ = NULL;

  // Update Mappoints
  for(set<Point*>::iterator it = mps.begin(); it != mps.end(); ++it)
  {
    (*it)->pos_ = (*it)->v_pt_->estimate();
    (*it)->v_pt_ = NULL;
  }

  // Remove Measurements with too large reprojection error
  double reproj_thresh_2_squared = reproj_thresh_2*reproj_thresh_2;
  for(list<EdgeContainerSE3>::iterator it = edges.begin(); it != edges.end(); ++it)
  {
    if(it->edge->chi2() > reproj_thresh_2_squared) //*(1<<it->feature_->level))
    {
      map->removePtFrameRef(it->frame, it->feature);
      ++n_incorrect_edges_2;
    }
  }

  // TODO: delete points and edges!
  init_error = sqrt(init_error)*center_kf->cam_->errorMultiplier2();
  final_error = sqrt(final_error)*center_kf->cam_->errorMultiplier2();
}

void globalBA(Map* map)
{
  // init g2o
  g2o::SparseOptimizer optimizer;
  setupG2o(&optimizer);

  list<EdgeContainerSE3> edges;
  list< pair<FramePtr,Feature*> > incorrect_edges;

  // Go through all Keyframes
  size_t v_id = 0;
  double reproj_thresh_2 = Config::lobaThresh() / map->lastKeyframe()->cam_->errorMultiplier2();
  double reproj_thresh_1_squared = Config::poseOptimThresh() / map->lastKeyframe()->cam_->errorMultiplier2();
  reproj_thresh_1_squared *= reproj_thresh_1_squared;
  for(list<FramePtr>::iterator it_kf = map->keyframes_.begin();
      it_kf != map->keyframes_.end(); ++it_kf)
  {
    // New Keyframe Vertex
    g2oFrameSE3* v_kf = createG2oFrameSE3(it_kf->get(), v_id++, false);
    (*it_kf)->v_kf_ = v_kf;
    optimizer.addVertex(v_kf);
    for(Features::iterator it_ftr=(*it_kf)->fts_.begin(); it_ftr!=(*it_kf)->fts_.end(); ++it_ftr)
    {
      // for each keyframe add edges to all observed mapoints
      Point* mp = (*it_ftr)->point;
      if(mp == NULL)
        continue;
      g2oPoint* v_mp = mp->v_pt_;
      if(v_mp == NULL)
      {
        // mappoint-vertex doesn't exist yet. create a new one:
        v_mp = createG2oPoint(mp->pos_, v_id++, false);
        mp->v_pt_ = v_mp;
        optimizer.addVertex(v_mp);
      }

      // Due to merging of mappoints it is possible that references in kfs suddenly
      // have a very large reprojection error which may result in distorted results.
      Vector2d error = vk::project2d((*it_ftr)->f) - vk::project2d((*it_kf)->w2f(mp->pos_));
      if(error.squaredNorm() > reproj_thresh_1_squared)
        incorrect_edges.push_back(pair<FramePtr,Feature*>(*it_kf, *it_ftr));
      else
      {
        g2oEdgeSE3* e = createG2oEdgeSE3(v_kf, v_mp, vk::project2d((*it_ftr)->f),
                                         true,
                                         reproj_thresh_2*Config::lobaRobustHuberWidth());

        edges.push_back(EdgeContainerSE3(e, it_kf->get(), *it_ftr));
        optimizer.addEdge(e);
      }
    }
  }

  // Optimization
  double init_error=0.0, final_error=0.0;
  if(Config::lobaNumIter() > 0)
    runSparseBAOptimizer(&optimizer, Config::lobaNumIter(), init_error, final_error);

  // Update Keyframe and MapPoint Positions
  for(list<FramePtr>::iterator it_kf = map->keyframes_.begin();
        it_kf != map->keyframes_.end(); ++it_kf)
  {
    (*it_kf)->T_f_w_ = SE3( (*it_kf)->v_kf_->estimate().rotation(),
                            (*it_kf)->v_kf_->estimate().translation());
    (*it_kf)->v_kf_ = NULL;
    for(Features::iterator it_ftr=(*it_kf)->fts_.begin(); it_ftr!=(*it_kf)->fts_.end(); ++it_ftr)
    {
      Point* mp = (*it_ftr)->point;
      if(mp == NULL)
        continue;
      if(mp->v_pt_ == NULL)
        continue;       // mp was updated before
      mp->pos_ = mp->v_pt_->estimate();
      mp->v_pt_ = NULL;
    }
  }

  // Remove Measurements with too large reprojection error
  for(list< pair<FramePtr,Feature*> >::iterator it=incorrect_edges.begin();
      it!=incorrect_edges.end(); ++it)
    map->removePtFrameRef(it->first.get(), it->second);

  double reproj_thresh_2_squared = reproj_thresh_2*reproj_thresh_2;
  for(list<EdgeContainerSE3>::iterator it = edges.begin(); it != edges.end(); ++it)
  {
    if(it->edge->chi2() > reproj_thresh_2_squared)
    {
      map->removePtFrameRef(it->frame, it->feature);
    }
  }
}

void setupG2o(g2o::SparseOptimizer * optimizer)
{
  // TODO: What's happening with all this HEAP stuff? Memory Leak?
  optimizer->setVerbose(false);

#if SCHUR_TRICK
  // solver
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
  //linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#else
  g2o::BlockSolverX::LinearSolverType * linearSolver;
  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
  //linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif

  solver->setMaxTrialsAfterFailure(5);
  optimizer->setAlgorithm(solver);

  // setup camera
  g2o::CameraParameters * cam_params = new g2o::CameraParameters(1.0, Vector2d(0.,0.), 0.);
  cam_params->setId(0);
  if (!optimizer->addParameter(cam_params)) {
    assert(false);
  }
}

void
runSparseBAOptimizer(g2o::SparseOptimizer* optimizer,
                     unsigned int num_iter,
                     double& init_error, double& final_error)
{
  optimizer->initializeOptimization();
  optimizer->computeActiveErrors();
  init_error = optimizer->activeChi2();
  optimizer->optimize(num_iter);
  final_error = optimizer->activeChi2();
}

g2oFrameSE3*
createG2oFrameSE3(Frame* frame, size_t id, bool fixed)
{
  g2oFrameSE3* v = new g2oFrameSE3();
  v->setId(id);
  v->setFixed(fixed);
  v->setEstimate(g2o::SE3Quat(frame->T_f_w_.unit_quaternion(), frame->T_f_w_.translation()));
  return v;
}

g2oPoint*
createG2oPoint(Vector3d pos,
               size_t id,
               bool fixed)
{
  g2oPoint* v = new g2oPoint();
  v->setId(id);
#if SCHUR_TRICK
  v->setMarginalized(true);
#endif
  v->setFixed(fixed);
  v->setEstimate(pos);
  return v;
}

g2oEdgeSE3*
createG2oEdgeSE3( g2oFrameSE3* v_frame,
                  g2oPoint* v_point,
                  const Vector2d& f_up,
                  bool robust_kernel,
                  double huber_width,
                  double weight)
{
  g2oEdgeSE3* e = new g2oEdgeSE3();
  e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_point));
  e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_frame));
  e->setMeasurement(f_up);
  e->information() = weight * Eigen::Matrix2d::Identity(2,2);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();      // TODO: memory leak
  rk->setDelta(huber_width);
  e->setRobustKernel(rk);
  e->setParameterId(0, 0); //old: e->setId(v_point->id());
  return e;
}

} // namespace ba
} // namespace svo
