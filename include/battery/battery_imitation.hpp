#pragma once

#include <battery/battery_interface.hpp>

namespace battery_client {

class BatteryClientImitation : public BatteryClientInterface {
 public:
  BatteryClientImitation();
  double getVoltage() override;
  int getCharge() override;
};

}  // namespace battery_client
