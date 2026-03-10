#ifndef MULTICOST_ARRAY_H
#define MULTICOST_ARRAY_H

#include <array>
#include <memory>
#include <utility>
#include <vector>
#include "multicost.hpp"


class MulticostID;

class IMulticostArray : public std::enable_shared_from_this<IMulticostArray> {
public:
    virtual int compare(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2) = 0;
    virtual int compare(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, unsigned int index) = 0;
    virtual std::unique_ptr<MulticostID> identity() = 0;
    virtual std::unique_ptr<MulticostID> op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2) = 0;
    virtual void op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, const std::unique_ptr<MulticostID>& res) = 0;
    virtual std::unique_ptr<MulticostID>  op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, unsigned int index) = 0;
    virtual void op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, const std::unique_ptr<MulticostID>& res, unsigned int index) = 0;
    virtual std::unique_ptr<MulticostID> copy(const std::unique_ptr<MulticostID>& mid) = 0;
    virtual bool is_identity(const std::unique_ptr<MulticostID>& mid) = 0;
    virtual bool is_identity(const std::unique_ptr<MulticostID>& mid, unsigned int index) = 0;
    virtual unsigned int num_values() const = 0;
    virtual unsigned int allocated_size() const = 0;
    virtual unsigned int num_monoids() const = 0;

protected:
    std::unique_ptr<MulticostID> make_id(unsigned int id);

private:
    virtual void free(const MulticostID& id) = 0;

    friend MulticostID;
};


/** DO NOT CREATE THIS OBJECT DIRECTLY.
    CREATE FROM Multicost ARRAY.
    Multicost ID TO TRACK Multicosts IN THE POOL OF Multicost ARRAY.
*/
class MulticostID {
public:
    unsigned int get_id() const {
        return this->id;
    };

    ~MulticostID();

private:
    std::weak_ptr<IMulticostArray> multicost_array;
    unsigned int id;

    MulticostID(std::weak_ptr<IMulticostArray>multicost_array, unsigned int id) :
        multicost_array(multicost_array), id(id) {}

    friend IMulticostArray;
};

inline std::unique_ptr<MulticostID> IMulticostArray::make_id(unsigned int id) {
    return std::move(std::unique_ptr<MulticostID>(new MulticostID(shared_from_this(), id)));
};

inline MulticostID::~MulticostID() {
    if (auto m_array = multicost_array.lock()) {
        m_array->free(*this);
    }
};




template<typename T, unsigned int SIZE>
class MonoMulticostArray : public IMulticostArray {
public:
    MonoMulticostArray(MonoMulticostProps<T, SIZE> props) : props(props){};

    
    std::unique_ptr<MulticostID> make_multicost(std::array<T, SIZE>&& vals) {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            values[id] = std::move(vals);
            poolID.pop_back();
            return make_id(id);
        } else {
            values.emplace_back(std::move(vals));
            return make_id(values.size() - 1);
        }
    }

    void copy(const std::unique_ptr<MulticostID>& dest, const std::array<T, SIZE>& srcValues, unsigned int index)  {
        std::array<T, SIZE>& destValues = values[dest->get_id()];
        destValues[index] = srcValues[index];
    };



    std::unique_ptr<MulticostID> identity() override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            props.identity(values[id]);
            poolID.pop_back();
            return make_id(id);
        } else {
            std::array<T, SIZE> value = props.identity();
            values.emplace_back(std::move(value));
            return make_id(values.size() - 1);
        }  
    };



    bool is_identity(const std::unique_ptr<MulticostID>& id) override {
        return props.compare(values[id->get_id()], props.identity()) == 0;
    }



    bool is_identity(const std::unique_ptr<MulticostID>& id, unsigned int index) override {
        return props.compare(values[id->get_id()], props.identity(), index) == 0;
    }



    int compare(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2) override {
        return props.compare(values[id1->get_id()], values[id2->get_id()]);
    };



    int compare(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, unsigned int index) override {
        return props.compare(values[id1->get_id()], values[id2->get_id()], index);
    };



    std::unique_ptr<MulticostID> op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2) override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            props.op(values[id1->get_id()], values[id2->get_id()], values[id]);
            poolID.pop_back();
            return make_id(id);
        } else {
            std::array<T, SIZE> value = props.op(values[id1->get_id()], values[id2->get_id()]);
            values.emplace_back(std::move(value));
            return make_id(values.size() - 1);
        }
    };


    std::unique_ptr<MulticostID> op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, unsigned int index) override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            props.op(values[id1->get_id()], values[id2->get_id()], values[id], index);
            poolID.pop_back();
            return make_id(id);
        } else {
            std::array<T, SIZE> value = props.op(values[id1->get_id()], values[id2->get_id()], index);
            values.emplace_back(std::move(value));
            return make_id(values.size() - 1);
        }
    };


    void op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, const std::unique_ptr<MulticostID>& res) override {
        props.op(values[id1->get_id()], values[id2->get_id()], values[res->get_id()]);
    };

    
    void op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, const std::unique_ptr<MulticostID>& res, unsigned int index) override {
        props.op(values[id1->get_id()], values[id2->get_id()], values[res->get_id()], index);
    };


    std::unique_ptr<MulticostID> copy(const std::unique_ptr<MulticostID>& mid)  override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            values[id] = values[mid->get_id()];
            poolID.pop_back();
            return make_id(id);
        } else {
            values.push_back(values[mid->get_id()]);
            return make_id(values.size() - 1);
        }
    };



    const std::array<T, SIZE>& get_values(const std::unique_ptr<MulticostID>& id) const {
        return values[id->get_id()];
    };



    unsigned int num_values() const override {
        return this->values.size() - this->poolID.size();
    }



    unsigned int allocated_size() const override {
        return this->values.size();
    }



    unsigned int num_monoids() const override{
        return SIZE;
    }

private:
    std::vector<std::array<T, SIZE>> values;
    MonoMulticostProps<T, SIZE> props;
    std::vector<unsigned int> poolID;

    void free(const MulticostID &id) override {
        poolID.push_back(id.get_id());
    }
};



template<typename ...Ts>
class PolyMulticostArray : public IMulticostArray {
public:
    PolyMulticostArray(PolyMulticostProps<Ts...> props) : props(props){};

    
    std::unique_ptr<MulticostID> make_multicost(std::tuple<Ts...>&& vals) {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            values[id] = std::move(vals);
            poolID.pop_back();
            return make_id(id);
        } else {
            values.emplace_back(std::move(vals));
            return make_id(values.size() - 1);
        }
    }
    

    void copy(const std::unique_ptr<MulticostID>& dest, const std::tuple<Ts...>& srcValues, unsigned int index) {
        std::tuple<Ts...>& destValues = values[dest->get_id()];
        copy_index_impl(destValues, srcValues, index);
    };


    std::unique_ptr<MulticostID> identity() override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            props.identity(values[id]);
            poolID.pop_back();
            return make_id(id);
        } else {
            std::tuple<Ts...> value = props.identity();
            values.push_back(value);
            return make_id(values.size() - 1);
        }  
    };


    bool is_identity(const std::unique_ptr<MulticostID>& id) override {
        return props.compare(values[id->get_id()], props.identity()) == 0;
    }



    bool is_identity(const std::unique_ptr<MulticostID>& id, unsigned int index) override {
        return props.compare(values[id->get_id()], props.identity(), index) == 0;
    }



    int compare(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2) override {
        return props.compare(values[id1->get_id()], values[id2->get_id()]);
    };



    int compare(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, unsigned int index) override {
        return props.compare(values[id1->get_id()], values[id2->get_id()], index);
    };



    std::unique_ptr<MulticostID> op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2) override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            props.op(values[id1->get_id()], values[id2->get_id()], values[id]);
            poolID.pop_back();
            return make_id(id);
        } else {
            std::tuple<Ts...> value = props.op(values[id1->get_id()], values[id2->get_id()]);
            values.emplace_back(std::move(value));
            return make_id(values.size() - 1);
        }
    };



    std::unique_ptr<MulticostID> op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, unsigned int index) override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            props.op(values[id1->get_id()], values[id2->get_id()], values[id], index);
            poolID.pop_back();
            return make_id(id);
        } else {
            std::tuple<Ts...> value = props.op(values[id1->get_id()], values[id2->get_id()], index);
            values.emplace_back(std::move(value));
            return make_id(values.size() - 1);
        }
    };


    void op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, const std::unique_ptr<MulticostID>& res) override {
        props.op(values[id1->get_id()], values[id2->get_id()], values[res->get_id()]);
    };

    
    void op(const std::unique_ptr<MulticostID>& id1, const std::unique_ptr<MulticostID>& id2, const std::unique_ptr<MulticostID>& res, unsigned int index) override {
        props.op(values[id1->get_id()], values[id2->get_id()], values[res->get_id()], index);
    };


    std::unique_ptr<MulticostID> copy(const std::unique_ptr<MulticostID>& mid)  override {
        if (poolID.size() > 0) {
            unsigned int id = poolID[poolID.size() - 1];
            values[id] = values[mid->get_id()];
            poolID.pop_back();
            return make_id(id);
        } else {
            values.push_back(values[mid->get_id()]);
            return make_id(values.size() - 1);
        }
    };



    const std::tuple<Ts...>& get_values(const std::unique_ptr<MulticostID>& id) const {
        return values[id->get_id()];
    };



    unsigned int num_values() const override {
        return this->values.size() - this->poolID.size();
    }



    unsigned int allocated_size() const override {
        return this->values.size();
    }



    unsigned int num_monoids() const override{
        return this->size;
    }

private:
    std::vector<std::tuple<Ts...>> values;
    PolyMulticostProps<Ts...> props;
    std::vector<unsigned int> poolID;
    static constexpr unsigned int size = sizeof...(Ts);

    void free(const MulticostID &id) override {
        poolID.push_back(id.get_id());
    }

    template <unsigned int I = 0>
    void copy_index_impl(std::tuple<Ts...>& dest, const std::tuple<Ts...>& src, unsigned int index) {
        constexpr unsigned int size = sizeof...(Ts);
        if constexpr (I < size) {
            if (I == index) {
                std::get<I>(dest) = std::get<I>(src);
            } else {
                copy_index_impl<I + 1>(dest, src, index);
            }
        } else {
            std::cerr << "Invalid Index." << std::endl;
            exit(1);
        }
    }
};


#endif