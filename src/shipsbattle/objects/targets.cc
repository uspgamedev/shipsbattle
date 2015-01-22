#include <shipsbattle/objects/targets.h>
#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/damageablesystem.h>

#include <ugdk/action/3D/element.h>

using shipsbattle::components::subsystems::DamageableSystem;
using shipsbattle::components::Hull;

namespace shipsbattle {
namespace objects {

/***********************/
/// Target METHODS
Target::operator DamageableSystem*() const {
    if (!valid()) return nullptr;
    return system_;
}

/***********************/
/// TargetData METHODS
std::vector<Target> TargetData::GetPossibleTargets() const {
    std::vector<Target> targets;
    if (!valid()) return targets;
    auto systems = object_.lock()->component<Hull>()->GetAllSubsystems();
    for (auto sys : systems) {
        targets.emplace_back(object_, sys);
    }
    return targets;
}

void TargetData::ToggleTarget(const std::string& sys_name, bool selected) {
    auto skey = subsystems_.find(sys_name);
    bool in_subsystems = skey != subsystems_.end();
    if (selected && !in_subsystems) {
        subsystems_.insert(sys_name);
    }
    else if (!selected && in_subsystems) {
        subsystems_.erase(skey);
    }
}

std::vector<Target> TargetData::GetTargets() {
    std::vector<Target> targets;
    if (!valid()) return targets;
    auto hull = object_.lock()->component<Hull>();
    for (auto sys_name : subsystems_) {
        auto sys = hull->GetSubsystemByName(sys_name);
        targets.emplace_back(object_, sys);
    }
    return targets;
}

bool TargetData::IsTargeted(const std::string& sys_name) const {
    return subsystems_.count(sys_name) > 0;
}

bool TargetData::IsTargeted(components::subsystems::DamageableSystem* sys) const {
    return IsTargeted(sys->name());
}

TargetData::operator Target() const {
    if (!valid()) return Target(object_, nullptr);
    auto sys = object_.lock()->component<Hull>()->GetSubsystemByName("MainHull");
    return Target(object_, sys);
}

} // namespace objects
} // namespace shipsbattle
