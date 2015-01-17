#include <shipsbattle/objects/projectilemodel.h>

namespace shipsbattle {
namespace objects {

ProjectileModel::ProjectileModel(const std::string& name)
: name_(name), damage_(100), splash_radius_(1.0), armor_piercing_(1.0), shot_cost_(10), lifetime_(30), 
mass_(10), mesh_name_(""), linear_speed_(10), angular_speed_(5), motion_lifetime_(2.0)
{
    decayment_ = [](double dist, double radius) {
        return dist / radius;
    };
}
void ProjectileModel::set_armor_piercing(double pierce) {
    armor_piercing_ = pierce;
    if (armor_piercing_ > 1.0) armor_piercing_ = 1.0;
    if (armor_piercing_ < 0.0) armor_piercing_ = 0.0;
}

void ProjectileModel::OnFire(components::subsystems::DamageableSystem* target) {

}
void ProjectileModel::OnHit(Projectile& self, Ship& target, const std::vector<ugdk::action::mode3d::component::ContactPoint>& pts) {

}

} // namespace objects
} // namespace shipsbattle
