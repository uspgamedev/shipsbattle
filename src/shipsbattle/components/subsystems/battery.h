#ifndef SHIPSBATTLE_SUBSYSTEMS_BATTERY_H
#define SHIPSBATTLE_SUBSYSTEMS_BATTERY_H

#include <shipsbattle/components/subsystems/damageablesystem.h>

namespace shipsbattle {
namespace components {
class PowerSystem;

namespace subsystems {

class Battery : public DamageableSystem {
public:
    Battery(const std::string& name) : DamageableSystem(name), energy_(1000), max_energy_(1000) {}

    /** Gets the amount of energy currently stored in this battery */
    double energy() const { return energy_; }
    void set_energy(double current_energy);
    /** Gets the energy limit of this battery, that is, how much this can hold. */
    double max_energy() const { return max_energy_; }
    void set_max_energy(double max_energy) { max_energy_ = max_energy; }

protected:
    double energy_;
    double max_energy_;

    friend class components::PowerSystem;
};

inline void Battery::set_energy(double current_energy) {
    energy_ = current_energy;
    if (energy_ > max_energy_) energy_ = max_energy_;
    if (energy_ < 0.0) energy_ = 0.0;
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_BATTERY_H
