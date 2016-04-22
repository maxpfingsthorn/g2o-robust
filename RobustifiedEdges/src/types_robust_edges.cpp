/*
To the extent possible under law, Max Pfingsthorn has waived all 
copyright and related or neighboring rights to "Edges for G2O with 
individual robust kernels". This work is published from: Germany.
*/

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "edge_robustified.hpp"

typedef EdgeRobustified<g2o::EdgeSE2> RobustifiedEdgeSE2;
typedef EdgeRobustified<g2o::EdgeSE3> RobustifiedEdgeSE3;

namespace g2o {

G2O_REGISTER_TYPE_GROUP(robust_edges);

G2O_REGISTER_TYPE(EDGE_SE2_ROBUST, RobustifiedEdgeSE2);
G2O_REGISTER_TYPE(EDGE_SE3:QUAT_ROBUST, RobustifiedEdgeSE3);


}