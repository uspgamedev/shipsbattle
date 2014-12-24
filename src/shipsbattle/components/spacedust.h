#ifndef SHIPSBATTLE_COMPONENTS_SPACEDUST_H
#define SHIPSBATTLE_COMPONENTS_SPACEDUST_H

#include <ugdk/action/3D/component.h>

namespace Ogre {
class ParticleSystem;
}

namespace shipsbattle {
namespace components {

class SpaceDust : public ugdk::action::mode3d::Component {

public:
    SpaceDust() {}

    virtual std::type_index type() const override;
    
  protected:
    void OnTaken() override;

    Ogre::ParticleSystem* dust_system_;
};

inline std::type_index SpaceDust::type() const {
    return typeid(SpaceDust);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_SPACEDUST_H
