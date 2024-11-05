#pragma once

#include <led/led_interface.hpp>

namespace led_client {

class LedClientImitation : public LedClientInterface {
 public:
  LedClientImitation();
  void setLedState(bool ratio) override;
  bool getLedState() override;

 private:
  LedState state;
};

}  // namespace led_client
