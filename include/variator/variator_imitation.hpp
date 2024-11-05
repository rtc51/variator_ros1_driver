#pragma once

#include <variator/variator_interface.hpp>

namespace variator_client {

class VariatorClientImitation : public VariatorClientInterface {
 public:
  VariatorClientImitation();
  void setRatio(double ratio) override;
  double getRatio() override;

 private:
  VariatorState state;
};

}  // namespace variator_client
