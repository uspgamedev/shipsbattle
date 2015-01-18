#include <shipsbattle/components/subsystems/sensorarray.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <OgreVector3.h>
#include <BtOgreExtras.h>

using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {
namespace subsystems {

SensorArray::SensorArray(const std::string& name)
: PoweredSystem(name)
{
}

double SensorArray::NeedsRecharge() {
    return 0;
}
void SensorArray::OnRecharge(double energy) {
    
}

void SensorArray::Update(double dt) {
    
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
