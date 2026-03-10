#ifndef MULTICOST_PATHFIND_FACTORY_H
#define MULTICOST_PATHFIND_FACTORY_H

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include "multicost_pathfind.hpp"

class MulticostPathfindFactory {
public:
    using Creator = std::function<std::unique_ptr<IMulticostPathfind>()>;

    static MulticostPathfindFactory& getInstance() {
        static MulticostPathfindFactory instance;
        return instance;
    }

    /*

     name : The user defined algorithm alias
     creator : The function that creates an instance of the algorithm
     */
    void registerAlgorithm(const std::string& name, Creator creator) {
        creators_[name] = creator;
    }

    std::unique_ptr<IMulticostPathfind> create(const std::string& name) {
        auto it = creators_.find(name);
        if (it != creators_.end()) {
            return it->second();
        }
        return nullptr;
    }

    bool hasAlgorithm(const std::string& name) const {
        return creators_.find(name) != creators_.end();
    }

    std::vector<std::string> getAvailableAlgorithms() const {
        std::vector<std::string> names;
        for (const auto& pair : creators_) {
            names.push_back(pair.first);
        }
        return names;
    }

private:
    MulticostPathfindFactory() = default;
    ~MulticostPathfindFactory() = default;
    MulticostPathfindFactory(const MulticostPathfindFactory&) = delete;
    MulticostPathfindFactory& operator=(const MulticostPathfindFactory&) = delete;

    std::unordered_map<std::string, Creator> creators_;
};

class AlgorithmRegistrar {
public:
    AlgorithmRegistrar(const std::string& name, MulticostPathfindFactory::Creator creator) {
        MulticostPathfindFactory::getInstance().registerAlgorithm(name, creator);
    }
};

#define REGISTER_PATHFIND_ALGORITHM(name, class_name) \
    static AlgorithmRegistrar register_##class_name(name, []() -> std::unique_ptr<IMulticostPathfind> { \
        return std::make_unique<class_name>(); \
    })

#endif