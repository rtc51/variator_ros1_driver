#include <led/led_imitation.hpp>

namespace led_client {

LedClientImitation::LedClientImitation() : state({false}) {}

void LedClientImitation::setLedState(bool new_state) {
  state.state = new_state;
}

bool LedClientImitation::getLedState() { return state.state; }

}  // namespace led_client