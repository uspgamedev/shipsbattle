#ifndef SHIPSBATTLE_COMPONENTS_POWERSYSTEM_H
#define SHIPSBATTLE_COMPONENTS_POWERSYSTEM_H

#include <ugdk/action/3D/component.h>
#include <vector>
#include <unordered_map>


namespace shipsbattle {
namespace components {

namespace subsystems {
class PowerGenerator;
class Battery;
class PoweredSystem;
}

class PowerSystem : public ugdk::action::mode3d::Component {
public:
    PowerSystem() {}
    virtual ~PowerSystem();

    virtual std::type_index type() const override;

    void AddPowerGenerator(const std::shared_ptr<subsystems::PowerGenerator>& gen);
    const std::shared_ptr<subsystems::PowerGenerator>& GetPowerGenerator(size_t index);
    const std::shared_ptr<subsystems::PowerGenerator>& GetPowerGenerator(const std::string& name);
    size_t GetNumPowerGenerators() const { return generators_.size(); }

    void AddBattery(const std::shared_ptr<subsystems::Battery>& battery);
    const std::shared_ptr<subsystems::Battery>& GetBattery(size_t index);
    const std::shared_ptr<subsystems::Battery>& GetBattery(const std::string& name);
    size_t GetNumBatteries() const { return batteries_.size(); }

    void Update(double dt);

protected:
    void OnTaken() override;

    friend class subsystems::PoweredSystem;
    void RegisterPoweredSystem(subsystems::PoweredSystem* powered_sys);

    std::vector<std::shared_ptr<subsystems::PowerGenerator>>    generators_;
    std::unordered_map<std::string, size_t>    generator_indexes_;
    std::vector<std::shared_ptr<subsystems::Battery>>    batteries_;
    std::unordered_map<std::string, size_t>    battery_indexes_;

    std::vector<subsystems::PoweredSystem*>  systems_;
};

inline std::type_index PowerSystem::type() const {
    return typeid(PowerSystem);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_HULL_H
