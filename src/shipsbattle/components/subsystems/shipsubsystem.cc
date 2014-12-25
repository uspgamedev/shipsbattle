#include <shipsbattle/components/subsystems/shipsubsystem.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

void ShipSubsystem::set_position(const Ogre::Vector3& pos) { 
    //FIXME: we should check if pos is inside the mesh
    position_ = pos; 
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
