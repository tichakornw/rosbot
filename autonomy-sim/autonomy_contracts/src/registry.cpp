#include "autonomy_contracts/registry.hpp"

#include <algorithm>
#include <utility>

namespace autonomy_contracts
{

namespace
{

template<typename CreatorMap>
std::vector<std::string> namesFromMap(const CreatorMap & creators)
{
  std::vector<std::string> names;
  names.reserve(creators.size());
  for (const auto & entry : creators) {
    names.push_back(entry.first);
  }
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace

ComponentRegistry & ComponentRegistry::instance()
{
  static ComponentRegistry registry;
  return registry;
}

bool ComponentRegistry::registerPlanner(const std::string & name, PlannerCreator creator)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return planner_creators_.emplace(name, std::move(creator)).second;
}

bool ComponentRegistry::registerController(const std::string & name, ControllerCreator creator)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return controller_creators_.emplace(name, std::move(creator)).second;
}

bool ComponentRegistry::registerPerception(const std::string & name, PerceptionCreator creator)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return perception_creators_.emplace(name, std::move(creator)).second;
}

std::unique_ptr<IPlannerComponent> ComponentRegistry::createPlanner(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = planner_creators_.find(name);
  if (it == planner_creators_.end()) {
    return nullptr;
  }
  return it->second();
}

std::unique_ptr<IControllerComponent> ComponentRegistry::createController(
  const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = controller_creators_.find(name);
  if (it == controller_creators_.end()) {
    return nullptr;
  }
  return it->second();
}

std::unique_ptr<IPerceptionComponent> ComponentRegistry::createPerception(
  const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = perception_creators_.find(name);
  if (it == perception_creators_.end()) {
    return nullptr;
  }
  return it->second();
}

std::vector<std::string> ComponentRegistry::availablePlanners() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return namesFromMap(planner_creators_);
}

std::vector<std::string> ComponentRegistry::availableControllers() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return namesFromMap(controller_creators_);
}

std::vector<std::string> ComponentRegistry::availablePerceptions() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return namesFromMap(perception_creators_);
}

}  // namespace autonomy_contracts
