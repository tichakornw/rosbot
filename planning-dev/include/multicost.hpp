#ifndef MULTICOST_H
#define MULTICOST_H

#include <array>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <tuple>
#include <utility>


/***
    Mono Multicost Properties
    Used to deal with Multicost with same Monoid data types
    Note that the compare and the operation functions can be different
    Assumes that Multicost is implemented with an array
*/
template <typename T, unsigned int SIZE>
class MonoMulticostProps {
public:
    MonoMulticostProps( 
            std::array<T, SIZE> identity_multicost,
            std::array<std::function<int(T a, T b)>, SIZE> compares,
            std::array<std::function<T(T a, T b)>, SIZE> operators
        ) :
            identity_multicost(identity_multicost),
            compares(compares),
            operators(operators)   
    {};


    // Return a copy of identity
    std::array<T, SIZE> identity() const {
        return this->identity_multicost;
    }; 


    void identity(std::array<T, SIZE> &res) const {
        res = this->identity_multicost;
    }; 


    std::array<T, SIZE> op(const std::array<T, SIZE> &a, const std::array<T, SIZE> &b) const {
        std::array<T, SIZE> result;
        for (unsigned int i = 0; i < SIZE; ++i) {
            result[i] = operators[i](a[i], b[i]);
        }
        return result;
    }


    std::array<T, SIZE> op(const std::array<T, SIZE> &a, const std::array<T, SIZE> &b, unsigned int index) {
        std::array<T, SIZE> result = this->identity_multicost;
        result[index] = operators[index](a[index], b[index]);
        return result;
    }


    void op(const std::array<T, SIZE> &a, const std::array<T, SIZE> &b, std::array<T, SIZE> &res) {
        for (unsigned int i = 0; i < SIZE; ++i) {
            res[i] = operators[i](a[i], b[i]);
        }
    }


    void op(const std::array<T, SIZE> &a, const std::array<T, SIZE> &b, std::array<T, SIZE> &res, unsigned int index) {
        res[index] = operators[index](a[index], b[index]);
    }


    int compare(const std::array<T, SIZE> &a, const std::array<T, SIZE> &b) const {
        int comp = 0;
        for (unsigned int i = 0; i < SIZE; ++i) {
            comp = compares[i](a[i], b[i]);
            if (comp != 0) {
                return comp;
            }
        }
        return comp;
    }


    int compare(const std::array<T, SIZE> &a, const std::array<T, SIZE> &b, int index) const {
        return compares[index](a[index], b[index]);
    }


private:
    std::array<std::function<int(T a, T b)>, SIZE> compares;
    std::array<std::function<T(T a, T b)>, SIZE> operators;
    std::array<T, SIZE> identity_multicost;
};



/**
    Poly Multicost Properties
    Used to deal with Multicost with varying Monoid data types
    Assumes that Multicost is implemented with a tuple
    Can be slow when selecting a specific element (looping over each type until one is reached)
*/
template <typename ...Ts>
class PolyMulticostProps {
public:
    PolyMulticostProps( 
        Ts ...identities,
        std::function<int(Ts a, Ts b)> ...compares,
        std::function<Ts(Ts a, Ts b)> ...operators) :
            identity_multicost(identities...),
            compares(std::make_tuple(compares...)),
            operators(std::make_tuple(operators...))   
    {};


    // Return a copy of identity
    std::tuple<Ts...> identity() const {
        return this->identity_multicost;
    }; 


    void identity(std::tuple<Ts...> &res) const {
        res = this->identity_multicost;
    }; 


    std::tuple<Ts...> op(const std::tuple<Ts...> &a, const std::tuple<Ts...> &b) const {
        constexpr auto N = std::index_sequence_for<Ts...>{};
        return op_impl(a, b, N);
    }


    std::tuple<Ts...> op(const std::tuple<Ts...> &a, const std::tuple<Ts...> &b, unsigned int index) {
        constexpr unsigned int size = std::tuple_size<decltype(this->compares)>::value;
        
        std::tuple<Ts...> result;
        op_index_impl(a, b, result, index);
        
        return result;
    }

    void op(const std::tuple<Ts...> &a, const std::tuple<Ts...> &b, std::tuple<Ts...> &res) {
        constexpr auto N = std::index_sequence_for<Ts...>{};
        op_impl(a, b, res, N);
    }


    void op(const std::tuple<Ts...> &a, const std::tuple<Ts...> &b, std::tuple<Ts...> &res, unsigned int index) {
        constexpr unsigned int size = std::tuple_size<decltype(this->compares)>::value;
        op_index_impl(a, b, res, index);
    }

    int compare(const std::tuple<Ts...> &a, const std::tuple<Ts...> &b) const {
        constexpr auto N = std::index_sequence_for<Ts...>{};
        return compare_impl(a, b, N);
    }


    int compare(const std::tuple<Ts...> &a, const std::tuple<Ts...> &b, int index) const {
        constexpr unsigned int size = std::tuple_size<decltype(this->compares)>::value;

        return compare_index_impl(a, b, index);
    }


private:
    std::tuple<std::function<int(Ts a, Ts b)>...> compares;
    std::tuple<std::function<Ts(Ts a, Ts b)>...> operators;
    std::tuple<Ts...> identity_multicost;
    
    template <std::size_t... Is>
    std::tuple<Ts...> op_impl(const std::tuple<Ts...>& a, const std::tuple<Ts...>& b, std::index_sequence<Is...>) const {
        return std::make_tuple(std::get<Is>(operators)(std::get<Is>(a), std::get<Is>(b))...);
    }


    template <std::size_t... Is>
    void op_impl(const std::tuple<Ts...>& a, const std::tuple<Ts...>& b, std::tuple<Ts...> &res, std::index_sequence<Is...>) {
        res = std::make_tuple(std::get<Is>(operators)(std::get<Is>(a), std::get<Is>(b))...);
    }

    template <std::size_t... Is>
    int compare_impl(const std::tuple<Ts...>& a, const std::tuple<Ts...>& b, std::index_sequence<Is...>) const {
        int result = 0;
        // Use a lambda to allow short-circuiting
        ((result == 0 ? result = std::get<Is>(compares)(std::get<Is>(a), std::get<Is>(b)) : result), ...);
        return result;
    }

    template <unsigned int I = 0>
    void op_index_impl(const std::tuple<Ts...>& a, const std::tuple<Ts...>& b, std::tuple<Ts...> &res, unsigned int index) {
        constexpr unsigned int size = sizeof...(Ts);
        if constexpr (I < size) {
            if (I == index) {
                std::get<I>(res) = std::get<I>(operators)(std::get<I>(a), std::get<I>(b));
            } else {
                op_index_impl<I + 1>(a, b, res, index);
            }
        } else {
            std::cerr << "ERROR: [void Poly std::tuple::op_index_impl] index argument is invalid. Index = " << index << ". Max Size = " << size << "." << std::endl;
            exit(1);
        }
    }

    template <unsigned int I = 0>
    int compare_index_impl(const std::tuple<Ts...>& a, const std::tuple<Ts...>& b, unsigned int index) const {
        constexpr unsigned int size = sizeof...(Ts);
        if constexpr (I < size) {   
            if (I == index) {
                return std::get<I>(compares)(std::get<I>(a), std::get<I>(b));
            } else {
                return compare_index_impl<I + 1>(a, b, index);
            }
        } else {
            std::cerr << "ERROR: [int Poly std::tuple::compare_index_impl] index argument is invalid. Index = " << index << ". Max Size = " << size << "." << std::endl;
            exit(1);
        }
    }
};

#endif