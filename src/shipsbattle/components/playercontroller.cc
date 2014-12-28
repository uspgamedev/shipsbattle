#include <shipsbattle/components/playercontroller.h>
#include <ugdk/input/module.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/physics.h>
#include <ugdk/action/3D/camera.h>
#include <ugdk/action/3D/component/body.h>

#include <OgreSceneNode.h>

#include <iostream>

using std::cout;
using std::endl;

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
    }
    else if (ev.button == ugdk::input::MouseButton::RIGHT) {
        cout << "TARGET CYCLE" << endl;
    }
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
    body->Move(body->orientation() * Ogre::Vector3::UNIT_Z * static_cast<Ogre::Real>(speed_) * 50);
}

void PlayerController::OnTaken() {
    auto parent = owner();
    auto& scene = parent->scene();

    // set event listeners
    scene.event_handler().AddObjectListener(this);

    // set update task
    scene.AddTask(ugdk::system::Task(
    [this](double dt) {
        this->Update(dt);
    }));
}

} // namespace components
} // namespace shipsbattle
