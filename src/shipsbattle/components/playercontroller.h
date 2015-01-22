#ifndef SHIPSBATTLE_COMPONENTS_PLAYERCONTROLLER_H
#define SHIPSBATTLE_COMPONENTS_PLAYERCONTROLLER_H

#include <shipsbattle/components/updateablecomponent.h>
#include <ugdk/system/eventhandler.h>
#include <ugdk/input/events.h>

namespace shipsbattle {
namespace components {

class PlayerController : public UpdateableComponent,
                         public ugdk::system::Listener<ugdk::input::MouseWheelEvent>,
                         public ugdk::system::Listener<ugdk::input::MouseMotionEvent>,
                         public ugdk::system::Listener<ugdk::input::KeyPressedEvent>,
                         public ugdk::system::Listener<ugdk::input::MouseButtonPressedEvent>,
                         public ugdk::system::Listener<ugdk::input::JoystickButtonPressedEvent>,
                         public ugdk::system::Listener<ugdk::input::JoystickAxisEvent>,
                         public ugdk::system::Listener<ugdk::input::JoystickHatEvent>,
                         public ugdk::system::Listener<ugdk::input::JoystickConnectedEvent>,
                         public ugdk::system::Listener<ugdk::input::JoystickDisconnectedEvent>
{
public:
    PlayerController();

    virtual std::type_index type() const override;
    
    void Handle(const ugdk::input::MouseWheelEvent& ev);
    void Handle(const ugdk::input::MouseMotionEvent& ev);
    void Handle(const ugdk::input::KeyPressedEvent& ev);
    void Handle(const ugdk::input::MouseButtonPressedEvent& ev);
    void Handle(const ugdk::input::JoystickButtonPressedEvent& ev);
    void Handle(const ugdk::input::JoystickAxisEvent& ev);
    void Handle(const ugdk::input::JoystickHatEvent& ev);
    void Handle(const ugdk::input::JoystickConnectedEvent& ev);
    void Handle(const ugdk::input::JoystickDisconnectedEvent& ev);

    void Update(double dt) override;

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

#endif // SHIPSBATTLE_COMPONENTS_PLAYERCONTROLLER_H
