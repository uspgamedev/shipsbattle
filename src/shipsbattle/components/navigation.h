#ifndef SHIPSBATTLE_COMPONENTS_NAVIGATION_H
#define SHIPSBATTLE_COMPONENTS_NAVIGATION_H

#include <shipsbattle/components/updateablecomponent.h>
#include <shipsbattle/objects/targets.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>

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

    objects::TargetSet GetSensorObjects();
    std::shared_ptr<objects::TargetData> ToggleTarget(const std::string& target_name, bool selected);
    objects::TargetSet GetTargets();
    bool IsTargeted(const std::string& target_name) const;
    
protected:
    std::vector<std::shared_ptr<subsystems::SensorArray>>    sensors_;
    std::unordered_map<std::string, size_t>    sensor_indexes_;

    std::unordered_map<std::string, std::shared_ptr<objects::TargetData>> objects_;
    std::unordered_set<std::string> targets_;
};

inline std::type_index Navigation::type() const {
    return typeid(Navigation);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_TACTICAL_H
