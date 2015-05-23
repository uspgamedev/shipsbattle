#include <shipsbattle/components/playercontroller.h>
#include <shipsbattle/components/tactical.h>
#include <shipsbattle/components/navigation.h>
#include <shipsbattle/components/motion.h>
#include <shipsbattle/objects/targets.h>
#include <shipsbattle/components/subsystems/damageablesystem.h>

#include <ugdk/input/module.h>
#include <ugdk/input/joystick.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/physics.h>
#include <ugdk/action/3D/camera.h>

#include <OgreSceneNode.h>

#include <iostream>

using std::cout;
using std::endl;
using ugdk::input::Scancode;
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
    if (ev.scancode == Scancode::ESCAPE)
        scene.Finish();
    else if (ev.scancode == Scancode::NUMPAD_1)
        scene.physics()->set_debug_draw_enabled(!scene.physics()->debug_draw_enabled());
    else if (ev.scancode == Scancode::R) {
        speed_ += 0.1;
        if (speed_ > 1.0) speed_ = 1.0;
        cout << "Speed is " << speed_ << endl;
    }
    else if (ev.scancode == Scancode::F) {
        speed_ -= 0.1;
        if (speed_ < -1.0) speed_ = -1.0;
        cout << "Speed is " << speed_ << endl;
    }
}
void PlayerController::Handle(const ugdk::input::MouseButtonPressedEvent& ev) {
    if (ev.button == ugdk::input::MouseButton::LEFT) {
        cout << "FIRE" << endl;
        auto tact = owner()->component<Tactical>();
        auto nav = owner()->component<Navigation>();
        tact->FireAll(nav->GetTargets());
    }
    else if (ev.button == ugdk::input::MouseButton::RIGHT) {
        cout << "TARGET CYCLE" << endl;
        auto nav = owner()->component<Navigation>();
        for (auto tdata : nav->GetTargets()) {

            bool mark_next = false;
            bool cycled = false;
            std::string prev_name;
            for (auto sys : tdata->GetPossibleTargets()) {
                if (mark_next) {
                    tdata->ToggleTarget(prev_name, false);
                    tdata->ToggleTarget(sys->name(), true);
                    cout << "New Target: " << tdata->object()->name() << ":" << sys->name() << " (old: " << prev_name << ")" << endl;
                    cycled = true;
                    break;
                }
                prev_name = sys->name();
                mark_next = tdata->IsTargeted(prev_name);
            }
            if (!cycled) {
                tdata->ToggleTarget(prev_name, false);
                auto sys = tdata->GetPossibleTargets().front();
                tdata->ToggleTarget(sys->name(), true);
                cout << "New Target: " << tdata->object()->name() << ":" << sys->name() << " (old: " << prev_name << ")" << endl;
            }

        }
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
    auto& keys = ugdk::input::manager()->keyboard();
    
    Ogre::Vector3 rotateAxis = Ogre::Vector3::ZERO;
    rotateAxis.x += (keys.IsDown(Scancode::S)) ? 1.0 : ((keys.IsDown(Scancode::W)) ? -1.0 : 0.0);
    rotateAxis.y += (keys.IsDown(Scancode::A)) ? 1.0 : ((keys.IsDown(Scancode::D)) ? -1.0 : 0.0);
    rotateAxis.z += (keys.IsDown(Scancode::E)) ? 1.0 : ((keys.IsDown(Scancode::Q)) ? -1.0 : 0.0);
    rotateAxis.normalise();

    Ogre::Vector3 moveToDir = Ogre::Vector3::ZERO;
    moveToDir.x += (keys.IsDown(Scancode::G)) ? 1.0 : ((keys.IsDown(Scancode::J)) ? -1.0 : 0.0);
    moveToDir.y += (keys.IsDown(Scancode::U)) ? 1.0 : ((keys.IsDown(Scancode::T)) ? -1.0 : 0.0);
    moveToDir.z += (keys.IsDown(Scancode::Y)) ? 1.0 : ((keys.IsDown(Scancode::H)) ? -1.0 : 0.0);
    moveToDir.normalise();

    auto motion = owner()->component<shipsbattle::components::Motion>();
    motion->TurnAround(rotateAxis, speed_, 0.1);
    motion->MoveTowards(moveToDir, speed_, 0.1);
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
