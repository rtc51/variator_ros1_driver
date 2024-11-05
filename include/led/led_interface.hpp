#pragma once

namespace led_client {

struct LedState {
  bool state;
};

class LedClientInterface {
 public:
  virtual void setLedState(bool ratio) = 0;
  virtual bool getLedState() = 0;

  virtual ~LedClientInterface() {}
};
}  // namespace led_client
