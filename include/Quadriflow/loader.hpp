#ifndef __LOADER_H
#define __LOADER_H

#include <Eigen/Core>
#include <vector>

namespace CadUSD::qflow {

using namespace Eigen;

void load(const char* filename, MatrixXd& V, MatrixXi& F);

} // namespace CadUSD::qflow

#endif
