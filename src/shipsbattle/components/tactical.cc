#include <shipsbattle/components/tactical.h>
#include <shipsbattle/components/subsystems/weapon.h>
#include <shipsbattle/objects/targets.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/debug/log.h>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::Weapon;


namespace shipsbattle {
namespace components {

void Tactical::AddWeapon(const shared_ptr<Weapon>& weapon) {
    if (weapon_indexes_.count(weapon->name()) > 0) {
        Log(LogLevel::WARNING, "Tactical System", "Weapon with name '" + weapon->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Weapon.");
        return;
    }
    weapons_.push_back(weapon);
    weapon_indexes_[weapon->name()] = weapons_.size() - 1;
    weapon->RegisteredTo(this);
}
const shared_ptr<Weapon>& Tactical::GetWeapon(size_t index) {
    return weapons_.at(index);
}
const shared_ptr<Weapon>& Tactical::GetWeapon(const std::string& name) {
    return GetWeapon(weapon_indexes_[name]);
}

void Tactical::Update(double dt) {
    for (auto weapon : weapons_) {
        weapon->Update(dt);
    }
}
void Tactical::FireAll(const std::vector<std::shared_ptr<objects::TargetData>>& targets) {
    std::vector<objects::Target> actual_targets;
    actual_targets.reserve(targets.size() * 2);
    for (auto tdata : targets) {
        auto systems = tdata->GetTargets();
        if (systems.size() == 0)
            actual_targets.push_back(*tdata);
        else
            actual_targets.insert(actual_targets.end(), systems.begin(), systems.end());
    }

    for (auto weapon : weapons_) {
        if (!weapon->CanFire()) continue;

        for (auto target : actual_targets) {
            if (weapon->TryFire(target)) break;
        }
    }
}

} // namespace components
} // namespace shipsbattle
