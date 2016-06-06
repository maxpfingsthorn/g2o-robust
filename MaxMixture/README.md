# Templated Implementation of MaxMixture Edge

This directory contains a new implementation of the [MaxMixture](https://github.com/agpratik/max-mixture) constraints by Agarwal and Olson. It improves over the original fork (maintained in MaxMixtureOriginal):

- Through templates, it is easy to add new subtypes. Right now, only template instances for EDGE_SE2 and EDGE_SE3:QUAT exist.
- A manager singleton updates the max components before every iteration, and also reinitializes the linear solver if necessary for "multimodal" edges (aka hyperedges). This is an important improvement, as it allows running these type of edges with a stock G2O install for as many iterations as needed. The original implementation only allowed to run the optimization for a single iteration, as the sparsity structure of the hessian may change between iterations, which G2O does not support per se. The structure is usually only computed once.

The names as exported by the plugin are compatible to the original edge types, so this should be a drop-in solution.

Note that this is still a hack. There is no direct support by G2O to notify the relevant classes (namely `g2o::OptimizationAlgorithm`) that the underlying sparsity pattern has changed.


## License

This implementation of Max-Mixture is licensed under the BSD license, as the original uses the same.

## Original References:

```
@INPROCEEDINGS{Olson-RSS-12, 
    AUTHOR    = {Edwin Olson AND Pratik Agarwal}, 
    TITLE     = {Inference on networks of mixtures for robust robot mapping}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2012}, 
    ADDRESS   = {Sydney, Australia}, 
    MONTH     = {July} 
}
```