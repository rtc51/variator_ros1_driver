#include <battery/battery_imitation.hpp>
#include <random>

namespace battery_client {

BatteryClientImitation::BatteryClientImitation() {}

double BatteryClientImitation::getVoltage() {
  std::random_device rd;
  std::default_random_engine reng(rd());
  std::uniform_real_distribution<double> dist(22.0, 25.5);
  return dist(reng);
}

int BatteryClientImitation::getCharge() {
  std::random_device rd;
  std::default_random_engine reng(rd());
  std::uniform_int_distribution<int> dist(80, 100);
  return dist(reng);
}

}  // namespace battery_client