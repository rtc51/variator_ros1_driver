#pragma once

namespace battery_client {

class BatteryClientInterface {
 public:
  virtual double getVoltage() = 0;
  virtual int getCharge() = 0;

  virtual ~BatteryClientInterface() {}
};
}  // namespace battery_client
