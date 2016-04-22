# Edges for G2O with individual robust kernels

This code is in the [public domain thanks to CC0](http://creativecommons.org/publicdomain/zero/1.0/). ![Public Domain](https://licensebuttons.net/p/zero/1.0/80x15.png)

This convenience plugin allows to specify per edge robust kernels.


For now, it only provides robustified variants for EDGE_SE2 and EDGE_SE3:QUAT types. The implementation is templated on the base edge type, so other edges are easily added.

Syntax:

EDGE_*_ROBUST <vertices> <kernel name> <kernel delta> <base edge info>

e.g.:
EDGE_SE2_ROBUST 0 1 Cauchy 1.0 1 0 0 10 0 0 10 0 10
----TAG-------- -v- -kn--- -d- -----edge info------

See also example g2o files in the main directory.