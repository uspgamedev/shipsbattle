#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/subhull.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/debug/log.h>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::DamageableSystem;
using shipsbattle::components::subsystems::SubHull;


namespace shipsbattle {
namespace components {

void Hull::AddSubHull(const shared_ptr<SubHull>& subhull) {
    if (subhull_indexes_.count(subhull->name()) > 0) {
        Log(LogLevel::WARNING, "Hull System", "SubHull with name '" + subhull->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this SubHull.");
        return;
    }
    subhulls_.push_back(subhull);
    subhull_indexes_[subhull->name()] = subhulls_.size() - 1;
    subhull->OnRegister(this);
}
const shared_ptr<SubHull>& Hull::GetSubHull(size_t index) {
    return subhulls_.at(index);
}
const shared_ptr<SubHull>& Hull::GetSubHull(const std::string& name) {
    return GetSubHull(subhull_indexes_[name]);
}

void Hull::OnTaken() {

}

void Hull::RegisterDamageableSystem(subsystems::DamageableSystem* dmgable_sys) {
    damageables_.push_back(dmgable_sys);
}

} // namespace components
} // namespace shipsbattle
