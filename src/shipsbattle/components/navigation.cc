#include <shipsbattle/components/navigation.h>
#include <shipsbattle/components/subsystems/sensorarray.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/debug/log.h>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::SensorArray;


namespace shipsbattle {
namespace components {

void Navigation::AddSensorArray(const shared_ptr<SensorArray>& sensor) {
    if (sensor_indexes_.count(sensor->name()) > 0) {
        Log(LogLevel::WARNING, "Navigation System", "SensorArray with name '" + sensor->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this SensorArray.");
        return;
    }
    sensors_.push_back(sensor);
    sensor_indexes_[sensor->name()] = sensors_.size() - 1;
    sensor->RegisteredTo(this);
}
const shared_ptr<SensorArray>& Navigation::GetSensorArray(size_t index) {
    return sensors_.at(index);
}
const shared_ptr<SensorArray>& Navigation::GetSensorArray(const std::string& name) {
    return GetSensorArray(sensor_indexes_[name]);
}

void Navigation::Update(double dt) {
    for (auto sensor : sensors_) {
        sensor->Update(dt);
    }
}

} // namespace components
} // namespace shipsbattle
