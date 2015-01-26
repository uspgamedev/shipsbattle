#ifndef SHIPSBATTLE_COMPONENTS_MOTION_H
#define SHIPSBATTLE_COMPONENTS_MOTION_H

#include <shipsbattle/components/updateablecomponent.h>
#include <vector>
#include <unordered_map>

namespace shipsbattle {
namespace components {
namespace subsystems {
class ImpulseEngine;
class Thruster;
}

class Motion : public UpdateableComponent {
public:
    Motion() {}

    virtual std::type_index type() const override;

    void AddImpulseEngine(const std::shared_ptr<subsystems::ImpulseEngine>& engine);
    const std::shared_ptr<subsystems::ImpulseEngine>& GetImpulseEngine(size_t index);
    const std::shared_ptr<subsystems::ImpulseEngine>& GetImpulseEngine(const std::string& name);
    size_t GetNumImpulseEngines() const { return impulse_engines_.size(); }

    void AddThruster(const std::shared_ptr<subsystems::Thruster>& thruster);
    const std::shared_ptr<subsystems::Thruster>& GetThruster(size_t index);
    const std::shared_ptr<subsystems::Thruster>& GetThruster(const std::string& name);
    size_t GetNumThrusters() const { return thrusters_.size(); }

    void Update(double dt) override;

protected:
    std::vector<std::shared_ptr<subsystems::ImpulseEngine>>    impulse_engines_;
    std::unordered_map<std::string, size_t>    impulse_indexes_;

    std::vector<std::shared_ptr<subsystems::Thruster>>    thrusters_;
    std::unordered_map<std::string, size_t>    thruster_indexes_;
};

inline std::type_index Motion::type() const {
    return typeid(Motion);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_MOTION_H
