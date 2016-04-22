/*
 * edge_se3Switchable.cpp
 *
 *  Created on: 17.10.2011
 *      Author: niko
 */

#include "edge_se3Switchable.h"
#include "vertex_switchLinear.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"

#include <GL/gl.h>


using namespace std;
using namespace Eigen;
using namespace g2o;


EdgeSE3Switchable::EdgeSE3Switchable() : BaseMultiEdge<6, Eigen::Isometry3d>() {
	information().setIdentity();
	
	resize(3);
	_jacobianOplus[0].resize(6,6); 
	_jacobianOplus[1].resize(6,6);
	_jacobianOplus[2].resize(6,1);
}

bool EdgeSE3Switchable::read(std::istream& is) {
	Vector7d meas;
	for (int i=0; i<7; i++) 
	  is >> meas[i];
	// normalize the quaternion to recover numerical precision lost by storing as human readable text
	Vector4d::MapType(meas.data()+3).normalize();
	setMeasurement(g2o::internal::fromVectorQT(meas));

	if (is.bad()) {
	  return false;
	}
	for ( int i=0; i<information().rows() && is.good(); i++)
	  for (int j=i; j<information().cols() && is.good(); j++){
		is >> information()(i,j);
		if (i!=j)
		  information()(j,i)=information()(i,j);
	  }
	if (is.bad()) {
	  //  we overwrite the information matrix with the Identity
	  information().setIdentity();
	} 
	return true;
}

bool EdgeSE3Switchable::write(std::ostream& os) const {
	Vector7d meas=g2o::internal::toVectorQT(_measurement);
	for (int i=0; i<7; i++) os  << meas[i] << " ";
	for (int i=0; i<information().rows(); i++)
	  for (int j=i; j<information().cols(); j++) {
		os <<  information()(i,j) << " ";
	  }
	return os.good();
}

void EdgeSE3Switchable::computeError() {
	g2o::VertexSE3 *from = static_cast<g2o::VertexSE3*>(_vertices[0]);
	g2o::VertexSE3 *to   = static_cast<g2o::VertexSE3*>(_vertices[1]);
	VertexSwitchLinear *sw   = static_cast<VertexSwitchLinear*>(_vertices[2]);
	
	Eigen::Isometry3d delta=_inverseMeasurement * from->estimate().inverse() * to->estimate();
	_error= sw->estimate() * g2o::internal::toVectorMQT(delta);
}

bool EdgeSE3Switchable::setMeasurementFromState(){
	g2o::VertexSE3 *from = static_cast<g2o::VertexSE3*>(_vertices[0]);
	g2o::VertexSE3 *to   = static_cast<g2o::VertexSE3*>(_vertices[1]);
	Eigen::Isometry3d delta = from->estimate().inverse() * to->estimate();
	setMeasurement(delta);
	return true;
}

void EdgeSE3Switchable::linearizeOplus(){

	// BaseBinaryEdge<6, Eigen::Isometry3d, VertexSE3, VertexSE3>::linearizeOplus();
	// return;

	g2o::VertexSE3 *from = static_cast<g2o::VertexSE3*>(_vertices[0]);
	g2o::VertexSE3 *to   = static_cast<g2o::VertexSE3*>(_vertices[1]);
	Eigen::Isometry3d E;
	const Eigen::Isometry3d& Xi=from->estimate();
	const Eigen::Isometry3d& Xj=to->estimate();
	const Eigen::Isometry3d& Z=_measurement;
	g2o::internal::computeEdgeSE3Gradient(E, _jacobianOplus[0] , _jacobianOplus[1], Z, Xi, Xj);
	
	// make 3rd jacobian wrt switch variable
	VertexSwitchLinear* sw = static_cast<VertexSwitchLinear*>(_vertices[2]);
	
	_jacobianOplus[0] *= sw->estimate();
	_jacobianOplus[1] *= sw->estimate();
	
    // derivative w.r.t switch vertex
    _jacobianOplus[2].setZero();

    Eigen::Isometry3d delta=_inverseMeasurement * from->estimate().inverse() * to->estimate();
    _jacobianOplus[2] = g2o::internal::toVectorMQT(delta) * sw->gradient();
    
    //~ // The analytic Jacobians assume the error in this special form (w beeing positive)
    //~ Eigen::Quaterniond r( g2o::internal::extractRotation( delta ) );
    //~ if (r.w() < 0.) {
       //~ _jacobianOplus[2].block<3,1>(3,0) *=  -1.;
	//~ }
}

void EdgeSE3Switchable::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
	g2o::VertexSE3 *from = static_cast<g2o::VertexSE3*>(_vertices[0]);
	g2o::VertexSE3 *to   = static_cast<g2o::VertexSE3*>(_vertices[1]);

	if (from_.count(from) > 0) {
	  to->setEstimate(from->estimate() * _measurement);
	} else
	  from->setEstimate(to->estimate() * _measurement.inverse());
	//cerr << "IE" << endl;
}


//~ 
//~ // ================================================
//~ EdgeSE3Switchable::EdgeSE3Switchable() : g2o::BaseMultiEdge<6, g2o::SE3Quat>()
//~ {
  //~ resize(3);
  //~ _jacobianOplus[0].resize(6,6); 
  //~ _jacobianOplus[1].resize(6,6);
  //~ _jacobianOplus[2].resize(6,1);
//~ 
//~ }
//~ // ================================================
//~ bool EdgeSE3Switchable::read(std::istream& is)
  //~ {
	//~ g2o::Vector7d m;
    //~ for (int i=0; i<7; i++)
       //~ is >> m[i];
    //~ Vector4d::MapType(m.data()+3).normalize();
    //~ g2o::SE3Quat m2;
    //~ m2.fromVector(m);
    //~ setMeasurement(m2);
//~ 
    //~ for (int i=0; i<6; i++)
      //~ for (int j=i; j<6; j++) {
        //~ is >> information()(i,j);
        //~ if (i!=j)
          //~ information()(j,i) = information()(i,j);
      //~ }
    //~ return true;
//~ 
  //~ }
//~ // ================================================
//~ bool EdgeSE3Switchable::write(std::ostream& os) const
//~ {
    //~ g2o::Vector7d p = measurement().toVector();
    //~ os << p.x() << " " << p.y() << " " << p.z();
    //~ for (int i = 0; i < 6; ++i)
      //~ for (int j = i; j < 6; ++j)
        //~ os << " " << information()(i, j);
    //~ return os.good();
//~ }
//~ 
//~ 
// forward declaration for the analytic jacobian
namespace g2o
{
  void  jacobian_3d_qman ( Matrix<double, 6, 6> &  Ji , Matrix<double, 6, 6> &  Jj, const double&  z11 , const double&  z12 , const double&  z13 , const double&  z14 , const double&  z21 , const double&  z22 , const double&  z23 , const double&  z24 , const double&  z31 , const double&  z32 , const double&  z33 , const double&  z34 , const double&  xab11 , const double&  xab12 , const double&  xab13 , const double&  xab14 , const double&  xab21 , const double&  xab22 , const double&  xab23 , const double&  xab24 , const double&  xab31 , const double&  xab32 , const double&  xab33 , const double&  xab34 );
}
//~ 
//~ 
//~ // ================================================
//~ void EdgeSE3Switchable::linearizeOplus()
//~ {
//~ 
    //~ g2o::deprecated::VertexSE3* from = static_cast<g2o::deprecated::VertexSE3*>(_vertices[0]);
    //~ g2o::deprecated::VertexSE3* to = static_cast<g2o::deprecated::VertexSE3*>(_vertices[1]);
    //~ const VertexSwitchLinear* vSwitch = static_cast<const VertexSwitchLinear*>(_vertices[2]);
//~ 
    //~ Matrix3d izR        = measurement().inverse().rotation().toRotationMatrix();
    //~ const Vector3d& izt = measurement().inverse().translation();
//~ 
    //~ g2o::SE3Quat iXiXj         = from->estimate().inverse() * to->estimate();
    //~ Matrix3d iRiRj        = iXiXj.rotation().toRotationMatrix();
    //~ const Vector3d& ititj = iXiXj.translation();
//~ 
    //~ Matrix<double, 6, 6> Ji, Jj;
//~ 
    //~ g2o::deprecated::jacobian_3d_qman ( Ji, Jj,
              //~ izR(0,0), izR(0,1), izR(0,2), izt(0),
              //~ izR(1,0), izR(1,1), izR(1,2), izt(1),
              //~ izR(2,0), izR(2,1), izR(2,2), izt(2),
              //~ iRiRj(0,0), iRiRj(0,1), iRiRj(0,2), ititj(0),
              //~ iRiRj(1,0), iRiRj(1,1), iRiRj(1,2), ititj(1),
              //~ iRiRj(2,0), iRiRj(2,1), iRiRj(2,2), ititj(2));
//~ 
    //~ _jacobianOplus[0] = Ji;
    //~ _jacobianOplus[1] = Jj;
//~ 
//~ 
    //~ _jacobianOplus[0]*=vSwitch->estimate();
    //~ _jacobianOplus[1]*=vSwitch->estimate();
//~ 
//~ 
    //~ // derivative w.r.t switch vertex
    //~ _jacobianOplus[2].setZero();
//~ 
    //~ g2o::SE3Quat delta = measurement().inverse() * (from->estimate().inverse()*to->estimate());
    //~ ErrorVector error;
    //~ error.head<3>() = delta.translation();
    //~ // The analytic Jacobians assume the error in this special form (w beeing positive)
    //~ if (delta.rotation().w() < 0.)
      //~ error.tail<3>() =  - delta.rotation().vec();
    //~ else
      //~ error.tail<3>() =  delta.rotation().vec();
//~ 
    //~ _jacobianOplus[2] = error * vSwitch->gradient();
//~ 
//~ }
//~ 
//~ 
//~ // ================================================
//~ void EdgeSE3Switchable::computeError()
//~ {
//~ 
    //~ const VertexSwitchLinear* v3 = static_cast<const VertexSwitchLinear*>(_vertices[2]);
//~ 
//~ 
    //~ const g2o::deprecated::VertexSE3* v1 = dynamic_cast<const g2o::deprecated::VertexSE3*>(_vertices[0]);
    //~ const g2o::deprecated::VertexSE3* v2 = dynamic_cast<const g2o::deprecated::VertexSE3*>(_vertices[1]);
    //~ g2o::SE3Quat delta = measurement().inverse() * (v1->estimate().inverse()*v2->estimate());
    //~ _error.head<3>() = delta.translation()* v3->estimate();
    //~ // The analytic Jacobians assume the error in this special form (w beeing positive)
    //~ if (delta.rotation().w() < 0.)
      //~ _error.tail<3>() =  - delta.rotation().vec()* v3->estimate();
    //~ else
      //~ _error.tail<3>() =  delta.rotation().vec()* v3->estimate();;
//~ 
//~ }


#ifdef G2O_HAVE_OPENGL
  EdgeSE3SwitchableDrawAction::EdgeSE3SwitchableDrawAction(): DrawAction(typeid(EdgeSE3Switchable).name()){}

  g2o::HyperGraphElementAction* EdgeSE3SwitchableDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
               g2o::HyperGraphElementAction::Parameters* /*params_*/){
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE3Switchable* e =  static_cast<EdgeSE3Switchable*>(element);


    g2o::VertexSE3* fromEdge = static_cast<g2o::VertexSE3*>(e->vertices()[0]);
    g2o::VertexSE3* toEdge   = static_cast<g2o::VertexSE3*>(e->vertices()[1]);
    VertexSwitchLinear* s   = static_cast<VertexSwitchLinear*>(e->vertices()[2]);

    glColor3f(s->estimate()*1.0,s->estimate()*0.1,s->estimate()*0.1);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),fromEdge->estimate().translation().z());
    glVertex3f(toEdge->estimate().translation().x(),toEdge->estimate().translation().y(),toEdge->estimate().translation().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif
