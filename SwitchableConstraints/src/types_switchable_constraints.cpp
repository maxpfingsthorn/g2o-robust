#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include "types_switchable_constraints.h"



namespace g2o {

G2O_REGISTER_TYPE_GROUP(switchable_constraints);

G2O_REGISTER_TYPE(VERTEX_SWITCH, VertexSwitchLinear);
G2O_REGISTER_TYPE(EDGE_SWITCH_PRIOR, EdgeSwitchPrior);
G2O_REGISTER_TYPE(EDGE_SE2_SWITCHABLE, EdgeSE2Switchable);
G2O_REGISTER_TYPE(EDGE_SE3:QUAT_SWITCHABLE, EdgeSE3Switchable);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(EdgeSE2SwitchableDrawAction);
G2O_REGISTER_ACTION(EdgeSE3SwitchableDrawAction);
#endif

}

// using namespace g2o;


// static void  init_types_veloc(void) __attribute((constructor));
// static void  init_types_veloc(void)
// {
//   Factory* factory = Factory::instance();
//   factory->registerType("EDGE_SWITCH_PRIOR", new HyperGraphElementCreator<EdgeSwitchPrior>);
//   factory->registerType("EDGE_SE2_SWITCHABLE", new HyperGraphElementCreator<EdgeSE2Switchable>);
//   factory->registerType("EDGE_SE3:QUAT_SWITCHABLE", new HyperGraphElementCreator<EdgeSE3Switchable>);
//   factory->registerType("VERTEX_SWITCH", new HyperGraphElementCreator<VertexSwitchLinear>);



//   g2o::HyperGraphActionLibrary* actionLib = g2o::HyperGraphActionLibrary::instance();

// #ifdef G2O_HAVE_OPENGL
//       actionLib->registerAction(new EdgeSE2SwitchableDrawAction);
//       actionLib->registerAction(new EdgeSE3SwitchableDrawAction);
// #endif


// }
