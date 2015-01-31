#include <shipsbattle/components/powersystem.h>
#include <shipsbattle/components/subsystems/powergenerator.h>
#include <shipsbattle/components/subsystems/battery.h>
#include <shipsbattle/components/subsystems/poweredsystem.h>

#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/debug/log.h>

#include <iostream>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::DamageableSystem;
using shipsbattle::components::subsystems::PowerGenerator;
using shipsbattle::components::subsystems::Battery;

namespace shipsbattle {
namespace components {


void PowerSystem::AddPowerGenerator(const shared_ptr<PowerGenerator>& gen) {
    if (generator_indexes_.count(gen->name()) > 0) {
        Log(LogLevel::WARNING, "Power System", "PowerGenerator with name '" + gen->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Generator.");
        return;
    }
    generators_.push_back(gen);
    generator_indexes_[gen->name()] = generators_.size() - 1;
    gen->RegisteredTo(this);
}
const shared_ptr<PowerGenerator>& PowerSystem::GetPowerGenerator(size_t index) {
    return generators_.at(index);
}
const shared_ptr<PowerGenerator>& PowerSystem::GetPowerGenerator(const std::string& name) {
    return GetPowerGenerator(generator_indexes_[name]);
}

void PowerSystem::AddBattery(const shared_ptr<Battery>& battery) {
    if (battery_indexes_.count(battery->name()) > 0) {
        Log(LogLevel::WARNING, "Power System", "Battery with name '" + battery->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Battery.");
        return;
    }
    batteries_.push_back(battery);
    battery_indexes_[battery->name()] = batteries_.size() - 1;
    battery->RegisteredTo(this);
}
const shared_ptr<Battery>& PowerSystem::GetBattery(size_t index) {
    return batteries_.at(index);
}
const shared_ptr<Battery>& PowerSystem::GetBattery(const std::string& name) {
    return GetBattery(battery_indexes_[name]);
}

void PowerSystem::Update(double dt) {
    // calculate total power production
    double total_output = 0.0;
    for (auto gen : generators_) {
        if (!gen->disabled()) {
            double gen_efficiency = gen->hitpoints() / gen->max_hitpoints();
            total_output += gen->output_rate() * dt * gen_efficiency;
        }
    }
    // calculate total power consumption
    double total_needed = 0.0;
    for (auto sys : systems_) {
        if (sys->disabled() || !sys->activated() || (sys->NeedsRecharge() <= 0.0)) continue;
        total_needed += sys->NeedsRecharge() * dt;
    }
    double energy_diff = total_output - total_needed;
    // calculate reserve power in batteries
    double total_stored = 0.0;
    double total_possible_store = 0.0;
    for (auto bat : batteries_) {
        if (bat->disabled()) continue;
        total_stored += bat->energy();
        total_possible_store += bat->max_energy();
    }
    // Update batteries stored power
    //   missing energy here is amount of energy we are missing:
    //   total_needed + missing = output + stored
    //   it should only be non-zero when batteries are/willbe empty.
    double missing_energy = (energy_diff < 0.0) ? total_stored + energy_diff : 0.0;
    missing_energy = (missing_energy < 0.0) ? -missing_energy : 0.0;
    for (auto bat : batteries_) {
        if (bat->disabled()) continue;
        if (missing_energy > 0.0) {
            bat->set_energy(0.0);
            continue;
        }
        double bat_factor = bat->max_energy() / total_possible_store;
        double energy_to_bat = energy_diff * bat_factor;
        bat->set_energy(bat->energy() + energy_to_bat);
    }

    // Recharge powered subsystems
    double power_factor = (total_needed - missing_energy) / total_needed;
    for (auto sys : systems_) {
        if (sys->disabled() || !sys->activated()) continue;
        double spent = sys->NeedsRecharge() * dt;
        double recharge_amount = spent * power_factor;
        sys->OnRecharge(recharge_amount);
    }
}


void PowerSystem::RegisterPoweredSystem(subsystems::PoweredSystem* powered_sys) {
    systems_.push_back(powered_sys);
}

} // namespace components
} // namespace shipsbattle
