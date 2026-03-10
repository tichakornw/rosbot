#ifndef MULTICOST_COMPUTE_H
#define MULTICOST_COMPUTE_H

#include "multicost_array.hpp"
#include <array>
#include <memory>
#include <tuple>

template<typename S>
class IMulticostCompute {
public:
    virtual std::unique_ptr<MulticostID> computeCost(S& a, S& b) = 0;
    virtual std::unique_ptr<MulticostID> computeCost(S& a, S& b,  unsigned int index) = 0;
    virtual void computeCost(S& a, S& b, const std::unique_ptr<MulticostID>& dest, unsigned int index) = 0;
};



template<typename S, typename ...Ts>
class PolyMulticostCompute : public IMulticostCompute<S> {
public:
    PolyMulticostCompute(
        std::shared_ptr<PolyMulticostArray<Ts...>> multicost_array,
        std::tuple<std::function<Ts(S& a, S& b)>...> computes
    ) : multicost_array(multicost_array), computes(computes) {};


    std::unique_ptr<MulticostID> computeCost(S& a, S& b, IMulticostArray& multicostArray) override {
        constexpr auto N = std::index_sequence_for<Ts...>{};
        return multicost_array->make_multicost(std::move(op_impl(a, b, N)));
    };


    std::unique_ptr<MulticostID> computeCost(S& a, S& b, unsigned int index) override {
        std::tuple<Ts...> result;
        op_index_impl(a, b, result, index);
        return multicost_array->make_multicost(std::move(result));
    };


    void computeCost(S& a, S& b, const std::unique_ptr<MulticostID>& dest, unsigned int index) {
        std::tuple<Ts...> result;
        op_index_impl(a, b, result, index);
        multicost_array->copy(dest, result, index);
    };


private:
    std::shared_ptr<PolyMulticostArray<Ts...>> multicost_array;
    std::tuple<std::function<Ts(S& a, S& b)>...> computes;
    
    template <std::size_t... Is>
    std::tuple<Ts...> op_impl(S& a, S& b, std::index_sequence<Is...>) const {
        return std::make_tuple(std::get<Is>(computes)(a, b)...);
    }

    template <unsigned int I = 0>
    void op_index_impl(S& a, S& b, std::tuple<Ts...>& result, unsigned int index) {
        constexpr unsigned int size = sizeof...(Ts);
        if constexpr (I < size) {
            if (I == index) {
                std::get<I>(result) = std::get<I>(computes)(a, b);
            } else {
                op_index_impl<I + 1>(index, result, a, b);
            }
        } else {
            std::cerr << "ERROR: index argument is invalid. Index = " << index << ". Max Size = " << size << "." << std::endl;
            exit(1);
        }
    }
};



template<typename S, typename T, unsigned int SIZE>
class MonoMulticostCompute : public IMulticostCompute<S> {
public:
    MonoMulticostCompute(
        std::shared_ptr<MonoMulticostArray<T, SIZE>> multicost_array, 
        std::array<std::function<T(S& a, S& b)>, SIZE> computes
    ) : multicost_array(multicost_array), computes(computes) {};


    std::unique_ptr<MulticostID> computeCost(S& a, S& b) override {
        std::array<T, SIZE> costs;
        for (unsigned i = 0; i < SIZE; ++i) {
            costs[i] = computes[i](a, b);
        }
        return multicost_array->make_multicost(std::move(costs));
    };

    
    std::unique_ptr<MulticostID> computeCost(S& a, S& b, unsigned int index) override {
        std::array<T, SIZE> costs;
        costs[index] = computes[index](a, b);
        return multicost_array->make_multicost(std::move(costs));
    };


    void computeCost(S& a, S& b, const std::unique_ptr<MulticostID>& dest, unsigned int index) {
        std::array<T, SIZE> costs;
        costs[index] = computes[index](a, b);
        multicost_array->copy(dest, costs, index);
    };


private:
    std::shared_ptr<MonoMulticostArray<T, SIZE>> multicost_array;
    std::array<std::function<T(S& a, S& b)>, SIZE> computes;

};

#endif