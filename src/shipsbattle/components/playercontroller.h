#ifndef SHIPSBATTLE_COMPONENTS_PLAYERCONTROLLER_H
#define SHIPSBATTLE_COMPONENTS_PLAYERCONTROLLER_H

#include <ugdk/action/3D/component.h>
#include <ugdk/system/eventhandler.h>
#include <ugdk/input/events.h>

namespace ugdk {
namespace input {
class MouseWheelEvent;
class MouseMotionEvent;
class KeyPressedEvent;
}
}

namespace shipsbattle {
namespace components {

class PlayerController : public ugdk::action::mode3d::Component,
                         public ugdk::system::Listener<ugdk::input::MouseWheelEvent>,
                         public ugdk::system::Listener<ugdk::input::MouseMotionEvent>,
                         public ugdk::system::Listener<ugdk::input::KeyPressedEvent>,
                         public ugdk::system::Listener<ugdk::input::MouseButtonPressedEvent>
{
public:
    PlayerController();

    virtual std::type_index type() const override;
    
    void Handle(const ugdk::input::MouseWheelEvent& ev);
    void Handle(const ugdk::input::MouseMotionEvent& ev);
    void Handle(const ugdk::input::KeyPressedEvent& ev);
    void Handle(const ugdk::input::MouseButtonPressedEvent& ev);

    void Update(double dt);

  protected:
    void OnTaken() override;

private:
    double speed_;
};

inline std::type_index PlayerController::type() const {
    return typeid(PlayerController);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_SPACEDUST_H
