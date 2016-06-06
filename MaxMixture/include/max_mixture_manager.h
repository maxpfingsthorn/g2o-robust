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

#pragma once

#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/sparse_optimizer.h"

#include "max_mixture_interface.h"

#include <boost/shared_ptr.hpp>

// singleton to manage maxmixture edges, update them, and, if necessary, reinitialize sover
class MaxMixtureManager : public g2o::HyperGraphAction {
public:
	MaxMixtureManager();

	virtual g2o::HyperGraphAction* operator()(const g2o::HyperGraph* graph, g2o::HyperGraphAction::Parameters* parameters = 0);

	void setGraph(g2o::OptimizableGraph*);
	void registerMaxMixture(MaxMixtureInterface* m);

	static boost::shared_ptr<MaxMixtureManager> instance();

protected:
	g2o::OptimizableGraph* _graph;
	std::list< MaxMixtureInterface* > _max_mixtures;
	std::set< MaxMixtureInterface* > _added_max_mixtures;

	static boost::shared_ptr<MaxMixtureManager> _instance;
};
