#ifndef SHIPSBATTLE_COMPONENTS_UPDATEABLECOMPONENT_H
#define SHIPSBATTLE_COMPONENTS_UPDATEABLECOMPONENT_H

#include <ugdk/action/3D/component.h>

namespace shipsbattle {
namespace components {

class UpdateableComponent : public ugdk::action::mode3d::Component {

public:
    virtual ~UpdateableComponent();

    virtual void Update(double dt) = 0;

  protected:
    void OnTaken() override;
};

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_UPDATEABLECOMPONENT_H
