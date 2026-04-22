#ifndef AUTONOMY_CONTRACTS__REGISTRY_HPP_
#define AUTONOMY_CONTRACTS__REGISTRY_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy_contracts/traits.hpp"
#include "autonomy_contracts/types.hpp"

namespace autonomy_contracts
{

template<typename InputT, typename OutputT>
class IPlannerComponentFor
{
public:
  virtual ~IPlannerComponentFor() = default;
  virtual OutputT plan(const InputT & input) = 0;
};

using IPlannerComponent = IPlannerComponentFor<PlanningInput2D, PlanningOutput2D>;

class IControllerComponent
{
public:
  virtual ~IControllerComponent() = default;
  virtual ControlOutput2D computeCommand(const ControlInput2D & input) = 0;
};

class IPerceptionComponent
{
public:
  virtual ~IPerceptionComponent() = default;
  virtual PerceptionOutput2D process(const PerceptionInput2D & input) = 0;
};

template<
  typename T,
  typename InputT = PlanningInput2D,
  typename OutputT = PlanningOutput2D>
class PlannerComponentModel : public IPlannerComponentFor<InputT, OutputT>
{
public:
  PlannerComponentModel()
  {
    validate_planner_component<T, InputT, OutputT>();
  }

  OutputT plan(const InputT & input) override
  {
    return component_.plan(input);
  }

private:
  T component_;
};

template<typename T>
class ControllerComponentModel : public IControllerComponent
{
public:
  ControllerComponentModel()
  {
    validate_controller_component<T>();
  }

  ControlOutput2D computeCommand(const ControlInput2D & input) override
  {
    return component_.computeCommand(input);
  }

private:
  T component_;
};

template<typename T>
class PerceptionComponentModel : public IPerceptionComponent
{
public:
  PerceptionComponentModel()
  {
    validate_perception_component<T>();
  }

  PerceptionOutput2D process(const PerceptionInput2D & input) override
  {
    return component_.process(input);
  }

private:
  T component_;
};

class ComponentRegistry
{
public:
  using PlannerCreator = std::function<std::unique_ptr<IPlannerComponent>()>;
  using ControllerCreator = std::function<std::unique_ptr<IControllerComponent>()>;
  using PerceptionCreator = std::function<std::unique_ptr<IPerceptionComponent>()>;

  static ComponentRegistry & instance();

  bool registerPlanner(const std::string & name, PlannerCreator creator);
  bool registerController(const std::string & name, ControllerCreator creator);
  bool registerPerception(const std::string & name, PerceptionCreator creator);

  std::unique_ptr<IPlannerComponent> createPlanner(const std::string & name) const;
  std::unique_ptr<IControllerComponent> createController(const std::string & name) const;
  std::unique_ptr<IPerceptionComponent> createPerception(const std::string & name) const;

  std::vector<std::string> availablePlanners() const;
  std::vector<std::string> availableControllers() const;
  std::vector<std::string> availablePerceptions() const;

private:
  ComponentRegistry() = default;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, PlannerCreator> planner_creators_;
  std::unordered_map<std::string, ControllerCreator> controller_creators_;
  std::unordered_map<std::string, PerceptionCreator> perception_creators_;
};

template<typename T>
bool registerPlannerComponent(const std::string & name)
{
  validate_planner_component<T>();
  return ComponentRegistry::instance().registerPlanner(
    name, []() {return std::make_unique<PlannerComponentModel<T>>();});
}

template<typename T>
bool registerControllerComponent(const std::string & name)
{
  validate_controller_component<T>();
  return ComponentRegistry::instance().registerController(
    name, []() {return std::make_unique<ControllerComponentModel<T>>();});
}

template<typename T>
bool registerPerceptionComponent(const std::string & name)
{
  validate_perception_component<T>();
  return ComponentRegistry::instance().registerPerception(
    name, []() {return std::make_unique<PerceptionComponentModel<T>>();});
}

}  // namespace autonomy_contracts

#endif  // AUTONOMY_CONTRACTS__REGISTRY_HPP_
