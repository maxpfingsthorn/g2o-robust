/*
To the extent possible under law, Max Pfingsthorn has waived all 
copyright and related or neighboring rights to "Edges for G2O with 
individual robust kernels". This work is published from: Germany.
*/

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

#include <string>

template < class ParentEdge >
class EdgeRobustified : public ParentEdge {
public:
	EdgeRobustified() {}
	virtual ~EdgeRobustified() {}

	bool read(std::istream& is) {
		is >> robustKernelName >> robustKernelDelta;

		if(! ParentEdge::read(is) ) {
			return false;
		}

		g2o::RobustKernel* k = g2o::RobustKernelFactory::instance()->construct(robustKernelName);
		if(k == NULL) {
			std::cerr << "EdgeRobustified::read: unknown kernel '" << robustKernelName << "'!" << std::endl;
			return false;
		}
		k->setDelta(robustKernelDelta);
		this->setRobustKernel(k);

		//std::cout << "EdgeRobustified::read: set robust kernel " << robustKernelName << " with delta " << robustKernelDelta << std::endl;

		return true;
	}
    bool write(std::ostream& os) const {
		os << robustKernelName << " " << robustKernelDelta << " ";
		return ParentEdge::write(os);
    }
   

private:
	std::string robustKernelName;
	double robustKernelDelta;
};
