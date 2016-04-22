/*
 * edge_se3Switchable.h
 *
 *  Created on: 17.10.2011
 *      Author: niko
 */

#ifndef EDGE_SE3SWITCHABLE_H_
#define EDGE_SE3SWITCHABLE_H_

#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

class EdgeSE3Switchable : public g2o::BaseMultiEdge<6, Eigen::Isometry3d>
{
   public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3Switchable();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      virtual void setMeasurement(const Eigen::Isometry3d& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
        Eigen::Map<const g2o::Vector7d> v(d);
        setMeasurement(g2o::internal::fromVectorQT(v));
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Eigen::Map<g2o::Vector7d> v(d);
        v = g2o::internal::toVectorQT(_measurement);
        return true;
      }

      void linearizeOplus();

      virtual int measurementDimension() const {return 7;}

      virtual bool setMeasurementFromState() ;

      virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& /*from*/, 
          g2o::OptimizableGraph::Vertex* /*to*/) { 
        return 1.;
      }

      virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to);
      
   protected:
      Eigen::Isometry3d _inverseMeasurement;
};


#ifdef G2O_HAVE_OPENGL
    class EdgeSE3SwitchableDrawAction: public g2o::DrawAction{
    public:
      EdgeSE3SwitchableDrawAction();
      virtual g2o::HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
              g2o::HyperGraphElementAction::Parameters* params_);
    };
#endif


#endif /* EDGE_SE3SWITCHABLE_H_ */
