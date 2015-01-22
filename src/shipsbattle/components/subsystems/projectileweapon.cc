#include <shipsbattle/components/subsystems/projectileweapon.h>
#include <shipsbattle/components/subsystems/damageablesystem.h>
#include <shipsbattle/objects/projectile.h>
#include <shipsbattle/objects/ship.h>
#include <shipsbattle/objects/targets.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/component/view.h>
#include <ugdk/action/3D/scene3d.h>

#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <BtOgreExtras.h>
#include <btBulletDynamicsCommon.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

ProjectileWeapon::ProjectileWeapon(const std::string& name)
    : Weapon(name), projectile_(objects::ProjectileModel(name + "Model")), launching_speed_(40.0)
{

}

bool ProjectileWeapon::Fire(const objects::Target& target) {
    if (energy_ < projectile_.shot_cost()) return false;

    energy_ -= projectile_.shot_cost();

    objects::Ship self(parent_->owner());
    auto& scene = self->scene();
    // direction the projectile is initially facing, taking weapon direction and ship orientation into account
    auto dir = self.body()->orientation() * direction_;
    dir.normalise();
    // position the projectile a little bit more than its radius forward, so that it doenst collide with our ship.
    double shot_radius = Ogre::MeshManager::getSingleton().getByName(projectile_.mesh_name())->getBoundingSphereRadius();
    auto pos = BtOgre::Convert::toOgre(world_position()) + (dir * static_cast<Ogre::Real>(shot_radius * 1.05));
    objects::Projectile shot = objects::Projectile(self, projectile_, target, pos, dir);
    
    auto shot_vel = self.body()->linear_velocity() + (dir * static_cast<Ogre::Real>(launching_speed_));
    shot.body()->set_linear_velocity(shot_vel);

    projectile_.OnFire(target);

    return true;
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
