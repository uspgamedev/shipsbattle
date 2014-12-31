#ifndef SHIPSBATTLE_SUBSYSTEMS_POWERGENERATOR_H
#define SHIPSBATTLE_SUBSYSTEMS_POWERGENERATOR_H

#include <shipsbattle/components/subsystems/damageablesystem.h>

namespace shipsbattle {
namespace components {
    class PowerSystem;

namespace subsystems {

class PowerGenerator : public DamageableSystem {
public:
    PowerGenerator(const std::string& name) : DamageableSystem(name), output_rate_(10.0) {}

    /** Gets the energy output rate per second */
    double output_rate() const { return output_rate_; }
    void set_output_rate(double energy_rate) { output_rate_ = energy_rate; }

protected:
    double output_rate_;

    friend class components::PowerSystem;
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_POWERGENERATOR_H
