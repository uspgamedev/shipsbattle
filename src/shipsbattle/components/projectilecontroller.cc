#include <shipsbattle/components/projectilecontroller.h>
#include <shipsbattle/components/subsystems/damageablesystem.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <BtOgreExtras.h>

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
    auto angle = projectile_.angular_speed() * dt;
    if (angle > angleToTarget)  angle = angleToTarget;

    perpendicular.normalize();
    dir.normalize();
    auto path = BtOgre::Convert::toOgre(dir.rotate(perpendicular, angle));
    path.normalise();

    body->set_orientation(path);
    body->Move(path * (projectile_.linear_speed() * projectile_.mass()));
}

void ProjectileController::OnTaken() {
    UpdateableComponent::OnTaken();
}

} // namespace components
} // namespace shipsbattle
