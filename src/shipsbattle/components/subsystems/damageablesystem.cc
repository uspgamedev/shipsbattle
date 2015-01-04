#include <shipsbattle/components/subsystems/damageablesystem.h>
#include <shipsbattle/components/hull.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/scene3d.h>

#include <btBulletCollisionCommon.h>
#include <BtOgreExtras.h>

#include <iostream>

using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {
namespace subsystems {

DamageableSystem::~DamageableSystem() {
    //delete volume_;
    delete shape_;
}

DamageableSystem::DamageableSystem(const std::string& name) : name_(name),
max_hitpoints_(1000), hitpoints_(1000.0), armor_rating_(0.0), required_(false), disabled_percentage_(0.25)
{
    SetVolume(0.1, btVector3(0.0, 0.0, 0.0));
}

void DamageableSystem::TakeDamage(double dmg, double piercing) {
    bool is_dmg = (dmg >= 0.0);
    dmg = (is_dmg) ? dmg : -dmg;
    double armor = armor_rating_ * (1.0 - piercing); //TODO: check
    if (armor > dmg) armor = dmg;
    double actual_dmg = dmg - armor;
    double armor_dmg = max_armor_rating_*(dmg / max_hitpoints_); //TODO: check

    if (!is_dmg) {
        actual_dmg = -actual_dmg;
        armor_dmg = -armor_dmg;
    }

    //std::cout << "Subsystem '" << name_ << "' damaged: " << std::endl;
    //std::cout << "  HP (r/a/chp): " << dmg << "/" << actual_dmg << "/" << hitpoints_ << std::endl;
    //std::cout << "  ARMOR (d/car/p): " << armor_dmg << "/" << armor_rating_ << "/" << piercing << std::endl;

    armor_rating_ -= armor_dmg;
    if (armor_rating_ > max_armor_rating_) armor_rating_ = max_armor_rating_;
    if (armor_rating_ < 0.0) armor_rating_ = 0.0;

    hitpoints_ -= actual_dmg;
    if (hitpoints_ > max_hitpoints_) hitpoints_ = max_hitpoints_;
    if (hitpoints_ < 0.0) {
        //TODO: KABUM
        std::cout << "Subsystem '" << name_ << "' KABUM" << std::endl;
        if (required_) {
            // kabum ship
            parent_->owner()->scene().DestroyAndRemoveElement(parent_->owner());
            std::cout << "DESTROYED AND REMOVED ELEMENT '" << parent_->owner()->name() << "'" << std::endl;
        }
    }
}

void DamageableSystem::SetVolume(double radius, const btVector3& pos) {
    shape_ = new btSphereShape(static_cast<btScalar>(radius));
    volume_ = new btCollisionObject();
    volume_->setCollisionShape(shape_);
    //TODO: check if position is inside the ship
    volume_->setWorldTransform(btTransform(btQuaternion::getIdentity(), pos));
    volume_->setUserPointer(this);
}

btVector3 DamageableSystem::position() const {
    return volume_->getWorldTransform().getOrigin();
}

btVector3 DamageableSystem::world_position() const {
    return position() + BtOgre::Convert::toBullet(parent_->owner()->component<Body>()->position());
}

double DamageableSystem::radius() const {
    return static_cast<double>(shape_->getRadius());
}

void DamageableSystem::RegisteredTo(ugdk::action::mode3d::Component* sys) {
    parent_ = sys;

    sys->owner()->component<Hull>()->RegisterDamageableSystem(this);

    this->OnRegister();
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
