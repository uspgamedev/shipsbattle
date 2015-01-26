#include <shipsbattle/components/motion.h>
#include <shipsbattle/components/subsystems/impulseengine.h>
#include <shipsbattle/components/subsystems/thruster.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/debug/log.h>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::ImpulseEngine;
using shipsbattle::components::subsystems::Thruster;

namespace shipsbattle {
namespace components {

void Motion::AddImpulseEngine(const shared_ptr<ImpulseEngine>& engine) {
    if (impulse_indexes_.count(engine->name()) > 0) {
        Log(LogLevel::WARNING, "Motion System", "ImpulseEngine with name '" + engine->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Impulse Engine.");
        return;
    }
    impulse_engines_.push_back(engine);
    impulse_indexes_[engine->name()] = impulse_engines_.size() - 1;
    engine->RegisteredTo(this);
}
const shared_ptr<ImpulseEngine>& Motion::GetImpulseEngine(size_t index) {
    return impulse_engines_.at(index);
}
const shared_ptr<ImpulseEngine>& Motion::GetImpulseEngine(const std::string& name) {
    return GetImpulseEngine(impulse_indexes_[name]);
}

void Motion::AddThruster(const shared_ptr<Thruster>& thruster) {
    if (thruster_indexes_.count(thruster->name()) > 0) {
        Log(LogLevel::WARNING, "Motion System", "Thruster with name '" + thruster->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Thruster.");
        return;
    }
    thrusters_.push_back(thruster);
    thruster_indexes_[thruster->name()] = thrusters_.size() - 1;
    thruster->RegisteredTo(this);
}
const shared_ptr<Thruster>& Motion::GetThruster(size_t index) {
    return thrusters_.at(index);
}
const shared_ptr<Thruster>& Motion::GetThruster(const std::string& name) {
    return GetThruster(thruster_indexes_[name]);
}

void Motion::Update(double dt) {

}


} // namespace components
} // namespace shipsbattle
