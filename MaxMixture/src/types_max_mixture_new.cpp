// Templated Max-Mixture plugin for g2o
// Copyright (C) 2016 M. Pfingsthorn
// Copyright (C) 2012 P. Agarwal, E. Olson, W. Burgard (Original Version)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "edge_max_mixture.hpp"

typedef EdgeMaxMixture<g2o::EdgeSE2> MaxMixtureEdgeSE2;
typedef EdgeMaxMixture<g2o::EdgeSE3> MaxMixtureEdgeSE3;

namespace g2o {

G2O_REGISTER_TYPE_GROUP(robust_edges);

G2O_REGISTER_TYPE(EDGE_SE2_MIXTURE, MaxMixtureEdgeSE2);
G2O_REGISTER_TYPE(EDGE_SE3:QUAT_MIXTURE, MaxMixtureEdgeSE3);


}