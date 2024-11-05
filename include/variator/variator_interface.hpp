#pragma once

namespace variator_client {

struct VariatorState {
  double ratio;
};

class VariatorClientInterface {
 public:
  virtual void setRatio(double ratio) = 0;
  virtual double getRatio() = 0;

  virtual ~VariatorClientInterface() {}
};
}  // namespace variator_client
