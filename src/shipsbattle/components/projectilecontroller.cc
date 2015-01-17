#include <shipsbattle/components/projectilecontroller.h>
#include <shipsbattle/components/subsystems/damageablesystem.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <BtOgreExtras.h>
#include <iostream>
using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {

ProjectileController::ProjectileController(const objects::Ship& parent_ship, const objects::ProjectileModel& projectile, subsystems::DamageableSystem* target)
    : parent_ship_(parent_ship), projectile_(projectile), target_(target), elapsed_(0.0)
{
}

void ProjectileController::Update(double dt) {
    if (elapsed_ > projectile_.motion_lifetime()) return;
    elapsed_ += dt;

    auto body = owner()->component<Body>();
    auto dir = BtOgre::Convert::toBullet(body->orientation() * Ogre::Vector3::UNIT_Z);

    // turn projectile
    auto target_pos = target_->world_position();
    auto target_dir = target_pos - BtOgre::Convert::toBullet(body->position());

    auto perpendicular = dir.cross(target_dir);
    auto angleToTarget = dir.angle(target_dir);
    dir.normalize();
    Ogre::Vector3 path;
    if (angleToTarget <= 0.0 || perpendicular.length2() <= 0.0) {
        // we're pointing exactly to target, no need to rotate direction.
        // besides, to do that we would need to perp.normalize(), and that will crash in this case (length 0).
        path = BtOgre::Convert::toOgre(dir);
    }
    else {
        auto angle = projectile_.angular_speed() * dt;
        if (angle > angleToTarget)  angle = angleToTarget;

        perpendicular.normalize();
        path = BtOgre::Convert::toOgre(dir.rotate(perpendicular, angle));
        path.normalise();
    }
    body->set_orientation(path);
    body->ApplyImpulse(path * (projectile_.linear_speed() * projectile_.mass()));

    /*
    *** TO FINISH WEAPON SYSTEM ISSUE
    * implement velocity push on projectile-ship collision (vai precisar de applyForceOnPosition)
    * implement bonus velocity damage for projectiles
    * fix OnHit to receive some gorram arguments
    * (for later) projectile weapons could have some capability to shoot leading the target
    */
}

void ProjectileController::OnTaken() {
    UpdateableComponent::OnTaken();
}

} // namespace components
} // namespace shipsbattle
