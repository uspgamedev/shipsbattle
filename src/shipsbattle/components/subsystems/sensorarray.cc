#include <shipsbattle/components/subsystems/sensorarray.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

SensorArray::SensorArray(const std::string& name)
: PoweredSystem(name), elapsed_(0.0), refresh_rate_(1.0), maximum_range_(1000.0)
{
}

double SensorArray::NeedsRecharge() {
    return efficiency() * energy_consumption();
}
void SensorArray::OnRecharge(double energy) {
    
}

void SensorArray::Update(double dt) {
    elapsed_ += dt;
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
