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

#include <string>
#include <iostream>

#include "max_mixture_interface.h"
#include "max_mixture_manager.h"


template < class ParentEdge >
class EdgeMaxMixture : public ParentEdge, public virtual MaxMixtureInterface {
public:
	EdgeMaxMixture() : current_max_component(0) {}
	virtual ~EdgeMaxMixture() {}

private:
	void copyFrom( ParentEdge* edge ) {
		std::copy( edge->vertices().begin(), edge->vertices().end(), this->vertices().begin() );

		this->setMeasurement( edge->measurement() );
		this->setInformation( edge->information() );
	}

	double negativeLogLikelihood(size_t i) {
		components[i]->computeError(); 
  		return negative_log_likelihood_offsets[i] + 0.5*components[i]->chi2();
	}

public:

	// new component edge must be initialized with vertices already!
	bool addComponent(double w, ParentEdge* edge) {
		weights.push_back(w);
		components.push_back(edge);
		negative_log_likelihood_offsets.push_back( -std::log(w) + 0.5 * edge->information().inverse().determinant() );

		if( components.size() == 1 ) { // initialize our vertices from the first component
			copyFrom( edge );
		}

		MaxMixtureManager::instance()->setGraph( this->graph() );
		MaxMixtureManager::instance()->registerMaxMixture(this);
	}

	size_t getCurrentComponentIndex() const {
		return current_max_component;
	}

	double getWeight(size_t c) const {
		return weights[c];
	}
	const ParentEdge* getComponent(size_t c) const {
		return components[c];
	}

	bool updateBelief() {
		if(components.size() == 0 ) {
			return false;
		}

		size_t max_component = 0;
		double max_component_nll = negativeLogLikelihood(0);

		for( size_t i = 1; i<components.size(); i++ ) {
			double nll = negativeLogLikelihood(i);
			if( max_component_nll > nll ) {
				max_component = i;
				max_component_nll = nll;
			}
		}

		//std::cerr << "EdgeMaxMixture: new: " << max_component << " (" << max_component_nll << ") old: " << current_max_component << " (" << negativeLogLikelihood(current_max_component) << ")" << std::endl; 

		bool need_reinit = false;
		if( max_component != current_max_component ) {
			for( size_t v=0; v<this->vertices().size(); v++ ) {
				if( components[max_component]->vertex(v) != components[current_max_component]->vertex(v) ) {
					need_reinit = true;
					break;
				}
			}
		}

		current_max_component = max_component;
		copyFrom( components[max_component] );

		return need_reinit;
	}


	bool read(std::istream& is) {
		size_t num_components;
		is >> num_components;

		weights.reserve(num_components);
		components.reserve(num_components);

		g2o::OptimizableGraph* graph = this->graph();

		// read and add components
		for(size_t i=0; i<num_components; i++) {
			std::string tag;
			double w;

			is >> tag >> w;

			if( edge_tag == "" ) {
				edge_tag = tag;
			}

			ParentEdge* comp = new ParentEdge();
			for( size_t v=0; v<comp->vertices().size(); v++ ) {
				int id;
				is >> id;

				comp->setVertex(v, graph->vertex(id) );
			}

			if(! comp->read(is) ) {
				return false;
			}

			addComponent(w, comp);
		}

		return is.good();
	}
    bool write(std::ostream& os) const {
		os << components.size();

		for(size_t i=0; i<components.size(); i++) {
			os << " " << edge_tag << " " << weights[i];
			for( size_t v=0; v<components[i]->vertices().size(); v++ ) {
				os << " " << components[i]->vertex(v)->id();
			}
			os << " ";
			if( !components[i]->write(os) ) {
				return false;
			}
		}
		return os.good();
    }
   

private:
	std::vector< double > weights, negative_log_likelihood_offsets;
	std::vector< ParentEdge* > components;
	size_t current_max_component;
	std::string edge_tag;
};
