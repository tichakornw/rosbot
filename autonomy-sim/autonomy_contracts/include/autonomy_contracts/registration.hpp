#ifndef AUTONOMY_CONTRACTS__REGISTRATION_HPP_
#define AUTONOMY_CONTRACTS__REGISTRATION_HPP_

#include "autonomy_contracts/registry.hpp"

#define AUTONOMY_CONTRACTS_CONCAT_IMPL(x, y) x##y
#define AUTONOMY_CONTRACTS_CONCAT(x, y) AUTONOMY_CONTRACTS_CONCAT_IMPL(x, y)

#define REGISTER_PLANNER_COMPONENT(name, class_name) \
  namespace { \
  [[maybe_unused]] const bool AUTONOMY_CONTRACTS_CONCAT(registered_planner_component_, __LINE__) = \
    ::autonomy_contracts::registerPlannerComponent<class_name>(name); \
  }

#define REGISTER_CONTROLLER_COMPONENT(name, class_name) \
  namespace { \
  [[maybe_unused]] const bool AUTONOMY_CONTRACTS_CONCAT(registered_controller_component_, __LINE__) = \
    ::autonomy_contracts::registerControllerComponent<class_name>(name); \
  }

#define REGISTER_PERCEPTION_COMPONENT(name, class_name) \
  namespace { \
  [[maybe_unused]] const bool AUTONOMY_CONTRACTS_CONCAT(registered_perception_component_, __LINE__) = \
    ::autonomy_contracts::registerPerceptionComponent<class_name>(name); \
  }

#endif  // AUTONOMY_CONTRACTS__REGISTRATION_HPP_
