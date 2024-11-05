#include <variator/variator_imitation.hpp>

namespace variator_client {

VariatorClientImitation::VariatorClientImitation() : state({1.0}) {}

void VariatorClientImitation::setRatio(double ratio) { state.ratio = ratio; }

double VariatorClientImitation::getRatio() { return state.ratio; }

}  // namespace variator_client