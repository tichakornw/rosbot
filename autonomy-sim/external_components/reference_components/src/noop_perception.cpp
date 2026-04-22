#include "autonomy_contracts/autonomy_contracts.hpp"

namespace autonomy_reference_components
{

class NoopPerception
{
public:
  autonomy_contracts::PerceptionOutput2D process(
    const autonomy_contracts::PerceptionInput2D & input)
  {
    (void)input;
    autonomy_contracts::PerceptionOutput2D output;
    output.success = true;
    return output;
  }
};

}  // namespace autonomy_reference_components

REGISTER_PERCEPTION_COMPONENT("noop_perception", autonomy_reference_components::NoopPerception)
