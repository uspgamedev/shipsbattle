#include <shipsbattle/components/projectilecontroller.h>
#include <shipsbattle/components/subsystems/damageablesystem.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <BtOgreExtras.h>

using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {

ProjectileController::ProjectileController(const objects::Ship& parent_ship, const objects::ProjectileModel& projectile, subsystems::DamageableSystem* target)
    : parent_ship_(parent_ship), projectile_(projectile), target_(target)
{
}

void ProjectileController::Update(double dt) {
    auto body = owner()->component<Body>();

    auto dir = body->orientation() * Ogre::Vector3::UNIT_Z;
    dir.normalise();

    // turn projectile
    auto target_pos = BtOgre::Convert::toOgre(target_->world_position());
    auto path = target_pos - body->position();
    auto rot = dir.getRotationTo(path);
    auto turn = rot * Ogre::Vector3::UNIT_Z;
    body->Rotate(turn.x, turn.y, turn.z);

    // boost projectile
    body->Move( dir * (projectile_.linear_speed() * projectile_.mass())  );
}

void ProjectileController::OnTaken() {
    UpdateableComponent::OnTaken();
}

} // namespace components
} // namespace shipsbattle
