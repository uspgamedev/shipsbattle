#include <shipsbattle/components/playercontroller.h>
#include <shipsbattle/components/tactical.h>
#include <ugdk/input/module.h>
#include <ugdk/input/joystick.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/physics.h>
#include <ugdk/action/3D/camera.h>
#include <ugdk/action/3D/component/body.h>

#include <OgreSceneNode.h>

#include <iostream>

using std::cout;
using std::endl;
using shipsbattle::components::Tactical;

namespace shipsbattle {
namespace components {

PlayerController::PlayerController() : speed_(0.0) {}

void PlayerController::Handle(const ugdk::input::MouseWheelEvent& ev) {
    auto& scene = owner()->scene();
    // zoom in/out camera with mouse wheel
    scene.camera()->SetDistance(scene.camera()->GetDistance() + ev.scroll.y * 10);
}
void PlayerController::Handle(const ugdk::input::MouseMotionEvent& ev) {
    // rotate camera with mouse motion
    owner()->scene().camera()->Rotate(-ev.motion.x, -ev.motion.y);
}
void PlayerController::Handle(const ugdk::input::KeyPressedEvent& ev) {
    auto& scene = owner()->scene();
    if (ev.scancode == ugdk::input::Scancode::ESCAPE)
        scene.Finish();
    else if (ev.scancode == ugdk::input::Scancode::NUMPAD_1)
        scene.physics()->set_debug_draw_enabled(!scene.physics()->debug_draw_enabled());
    else if (ev.scancode == ugdk::input::Scancode::E) {
        speed_ += 0.1;
        if (speed_ > 1.0) speed_ = 1.0;
        cout << "Speed is " << speed_ << endl;
    }
    else if (ev.scancode == ugdk::input::Scancode::Q) {
        speed_ -= 0.1;
        if (speed_ < -1.0) speed_ = -1.0;
        cout << "Speed is " << speed_ << endl;
    }
}
void PlayerController::Handle(const ugdk::input::MouseButtonPressedEvent& ev) {
    if (ev.button == ugdk::input::MouseButton::LEFT) {
        cout << "FIRE" << endl;
        auto tact = owner()->component<Tactical>();
        tact->FireAll(target_);
    }
    else if (ev.button == ugdk::input::MouseButton::RIGHT) {
        cout << "TARGET CYCLE" << endl;
    }
}

void PlayerController::Handle(const ugdk::input::JoystickButtonPressedEvent& ev) {
    cout << "joystick button " << ev.button << endl;
}
void PlayerController::Handle(const ugdk::input::JoystickAxisEvent& ev) {
    cout << "Axis " << ev.axis_id << " [" << ev.axis_status.Percentage() << "]" << endl;
}
void PlayerController::Handle(const ugdk::input::JoystickHatEvent& ev) {
    cout << "Hat " << ev.hat_id << " (" << ev.hat_status.IsUp() << ev.hat_status.IsRight() << ev.hat_status.IsDown() << ev.hat_status.IsLeft() << ")";
    cout << " (" << ev.hat_status.IsRightUp() << ev.hat_status.IsRightDown() << ev.hat_status.IsLeftDown() << ev.hat_status.IsLeftUp() << ")" << endl;
}

void PlayerController::Handle(const ugdk::input::JoystickConnectedEvent& ev) {
    auto joystick = ev.joystick.lock();
    joystick->event_handler().AddObjectListener(this);
    cout << "Joystick [" << joystick.get() << "] [" << joystick->NumAxes() << " axis] [" << joystick->NumHats() << " hats] ["
        << joystick->NumTrackballs() << " balls] [" << joystick->NumButtons() << " buttons]" << endl;
}
void PlayerController::Handle(const ugdk::input::JoystickDisconnectedEvent& ev) {
    cout << "joystick disconnected" << endl;
}


void PlayerController::Update(double dt) {
    auto& keyboard = ugdk::input::manager()->keyboard();
    auto body = owner()->component<ugdk::action::mode3d::component::Body>();

    Ogre::Vector3 rotate = Ogre::Vector3::ZERO;
    if (keyboard.IsDown(ugdk::input::Scancode::D))
        rotate.y += -1.0;
    else if (keyboard.IsDown(ugdk::input::Scancode::A))
        rotate.y += 1.0;
    if (keyboard.IsDown(ugdk::input::Scancode::W))
        rotate.x += -1.0;
    else if (keyboard.IsDown(ugdk::input::Scancode::S))
        rotate.x += 1.0;

    rotate.normalise();
    rotate = body->orientation() * rotate;
    rotate.normalise();
    rotate *= 30;
    //cout << "Rotation: (" << rotate.x << ", " << rotate.y << ", " << rotate.z << ")" << endl;
    
    body->Rotate(rotate.x, rotate.y, rotate.z);
    body->ApplyImpulse(body->orientation() * Ogre::Vector3::UNIT_Z * static_cast<Ogre::Real>(speed_) * 50);
}

void PlayerController::OnTaken() {
    auto parent = owner();
    auto& scene = parent->scene();

    // set event listeners
    scene.event_handler().AddObjectListener(this);
    
    // this sets update task
    UpdateableComponent::OnTaken();
}

} // namespace components
} // namespace shipsbattle
