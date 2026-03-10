#pragma once

#include <any>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace r2 {

struct FlagEvent {
    std::string name;
    std::any value;
};

class FlagBus {
public:
    using Callback = std::function<void(const FlagEvent&)>;

    void publish(const std::string& name, std::any value) {
        flags_[name] = value;
        FlagEvent event{name, value};
        auto iter = subscribers_.find(name);
        if (iter == subscribers_.end()) {
            return;
        }
        for (const auto& cb : iter->second) {
            cb(event);
        }
    }

    template <typename T>
    T get_or(const std::string& name, const T& default_value) const {
        auto iter = flags_.find(name);
        if (iter == flags_.end()) {
            return default_value;
        }
        try {
            return std::any_cast<T>(iter->second);
        } catch (const std::bad_any_cast&) {
            return default_value;
        }
    }

    void subscribe(const std::string& name, Callback callback) {
        subscribers_[name].push_back(std::move(callback));
    }

private:
    std::unordered_map<std::string, std::any> flags_;
    std::unordered_map<std::string, std::vector<Callback>> subscribers_;
};

}  // namespace r2
