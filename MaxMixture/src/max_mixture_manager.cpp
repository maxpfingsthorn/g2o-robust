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

#include "max_mixture_manager.h"

#include <iostream>

#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"

using namespace std;

boost::shared_ptr<MaxMixtureManager> MaxMixtureManager::_instance;

boost::shared_ptr<MaxMixtureManager> MaxMixtureManager::instance() {
	if(!_instance) {
		_instance = boost::shared_ptr<MaxMixtureManager>( new MaxMixtureManager() );
	}
	return _instance;
}

MaxMixtureManager::MaxMixtureManager() {
	_graph = NULL;
}
	
g2o::HyperGraphAction* MaxMixtureManager::operator()(const g2o::HyperGraph* graph, g2o::HyperGraphAction::Parameters* parameters) {
	if( graph != _graph ) {
		cerr << "didn't get the same graph!?!" << endl;
		return this;
	}

	int iteration = -1;

	{
		g2o::HyperGraphAction::ParametersIteration* pit = dynamic_cast<g2o::HyperGraphAction::ParametersIteration*>(parameters);
		if( pit == NULL ) {
			std::cerr << "MaxMixtureManager: Can't get iteration parameter" << std::endl;
			return this;
		}
		iteration = pit->iteration;
	}

	//cerr << "MaxMixtureManager: updating beliefs" << endl;
	bool reinitialize_solver = false;
	for( MaxMixtureInterface* m : _max_mixtures ) {
		if( m->updateBelief() ) {
			reinitialize_solver = true;
		}
	}

	if(reinitialize_solver && iteration >= 0) {
		const g2o::SparseOptimizer* sp = dynamic_cast<const g2o::SparseOptimizer*>(_graph);

		if( sp != NULL ) {
			//std::cerr << "MaxMixtureManager: called init!" << std::endl;

			g2o::OptimizationAlgorithm* algo = const_cast< g2o::OptimizationAlgorithm* >( sp->algorithm() );
			algo->init();
			if( iteration > 0 ) {
				g2o::OptimizationAlgorithmWithHessian* awh = dynamic_cast<g2o::OptimizationAlgorithmWithHessian*>(algo);
				if(awh != NULL)	{
					awh->buildLinearStructure();
				}
			}

		} else {
			std::cerr << "MaxMixtureManager: Graph is not a SparseOptimizer!" << std::endl;
		}
	}

	return this;
}

void MaxMixtureManager::setGraph(g2o::OptimizableGraph* g) {
	if( _graph == NULL ) {
		_graph = g;

		_graph->addPreIterationAction(this);
	}
}

void MaxMixtureManager::registerMaxMixture(MaxMixtureInterface* m) {
	if( _added_max_mixtures.find(m) == _added_max_mixtures.end() ) {
		_added_max_mixtures.insert(m);
		_max_mixtures.push_back(m);
	}
}