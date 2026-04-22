#ifndef AUTONOMY_CONTRACTS__TRAITS_HPP_
#define AUTONOMY_CONTRACTS__TRAITS_HPP_

#include <type_traits>
#include <utility>

#include "autonomy_contracts/types.hpp"

namespace autonomy_contracts
{

template<typename T, typename = void>
struct is_planner_component : std::false_type {};

template<typename T>
struct is_planner_component<
  T,
  std::void_t<decltype(std::declval<T &>().plan(std::declval<const PlanningInput2D &>()))>>
  : std::is_same<
      decltype(std::declval<T &>().plan(std::declval<const PlanningInput2D &>())),
      PlanningOutput2D> {};

template<typename T, typename = void>
struct is_controller_component : std::false_type {};

template<typename T>
struct is_controller_component<
  T,
  std::void_t<decltype(std::declval<T &>().computeCommand(
    std::declval<const ControlInput2D &>()))>>
  : std::is_same<
      decltype(std::declval<T &>().computeCommand(std::declval<const ControlInput2D &>())),
      ControlOutput2D> {};

template<typename T, typename = void>
struct is_perception_component : std::false_type {};

template<typename T>
struct is_perception_component<
  T,
  std::void_t<decltype(std::declval<T &>().process(
    std::declval<const PerceptionInput2D &>()))>>
  : std::is_same<
      decltype(std::declval<T &>().process(std::declval<const PerceptionInput2D &>())),
      PerceptionOutput2D> {};

template<typename T>
constexpr void validate_planner_component()
{
  static_assert(
    is_planner_component<T>::value,
    "Planner components must implement: PlanningOutput2D plan(const PlanningInput2D&)");
  static_assert(
    std::is_default_constructible<T>::value,
    "Registered planner components must be default constructible");
}

template<typename T>
constexpr void validate_controller_component()
{
  static_assert(
    is_controller_component<T>::value,
    "Controller components must implement: ControlOutput2D computeCommand(const ControlInput2D&)");
  static_assert(
    std::is_default_constructible<T>::value,
    "Registered controller components must be default constructible");
}

template<typename T>
constexpr void validate_perception_component()
{
  static_assert(
    is_perception_component<T>::value,
    "Perception components must implement: PerceptionOutput2D process(const PerceptionInput2D&)");
  static_assert(
    std::is_default_constructible<T>::value,
    "Registered perception components must be default constructible");
}

}  // namespace autonomy_contracts

#endif  // AUTONOMY_CONTRACTS__TRAITS_HPP_
