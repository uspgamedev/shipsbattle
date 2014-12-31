#ifndef SHIPSBATTLE_SUBSYSTEMS_POWEREDSYSTEM_H
#define SHIPSBATTLE_SUBSYSTEMS_POWEREDSYSTEM_H

#include <shipsbattle/components/subsystems/damageablesystem.h>

namespace shipsbattle {
namespace components {
    class PowerSystem;

namespace subsystems {

class PoweredSystem : public DamageableSystem {
public:
    
    virtual double NeedsRecharge() = 0;
    virtual void OnRecharge(double energy) = 0;

    /** If this subsystem is activated or not */
    bool activated() const { return activated_; }
    void set_activated(bool active) { activated_ = active; }
    /** Gets the desired efficiency (in percentage) of this system. 1.0 means 100% power - regular efficiency. */
    double efficiency() const { return efficiency_; }
    void set_efficiency(double effi) { efficiency_ = effi; }
    /** Gets the rate of energy consumption per second at regular efficiency*/
    double energy_consumption() const { return energy_consumption_; }
    void set_energy_consumption(double ec) { energy_consumption_ = ec; }

protected:
    bool activated_;
    double efficiency_;
    double energy_consumption_;

    PoweredSystem(const std::string& name);
    void RegisterToPowerSystem(PowerSystem* hull);

    void OnRegister() override;
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_POWEREDSYSTEM_H
