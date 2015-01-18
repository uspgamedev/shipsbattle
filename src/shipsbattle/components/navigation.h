#ifndef SHIPSBATTLE_COMPONENTS_NAVIGATION_H
#define SHIPSBATTLE_COMPONENTS_NAVIGATION_H

#include <shipsbattle/components/updateablecomponent.h>
#include <vector>
#include <unordered_map>

namespace shipsbattle {
namespace components {
namespace subsystems {
class SensorArray;
}

class Navigation : public UpdateableComponent {
public:
    Navigation() {}

    virtual std::type_index type() const override;

    void AddSensorArray(const std::shared_ptr<subsystems::SensorArray>& sensor);
    const std::shared_ptr<subsystems::SensorArray>& GetSensorArray(size_t index);
    const std::shared_ptr<subsystems::SensorArray>& GetSensorArray(const std::string& name);
    size_t GetNumSensorArrays() const { return sensors_.size(); }

    void Update(double dt) override;

protected:
    std::vector<std::shared_ptr<subsystems::SensorArray>>    sensors_;
    std::unordered_map<std::string, size_t>    sensor_indexes_;
};

inline std::type_index Navigation::type() const {
    return typeid(Navigation);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_TACTICAL_H
