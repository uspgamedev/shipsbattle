#include <shipsbattle/objects/projectilemodel.h>

namespace shipsbattle {
namespace objects {

ProjectileModel::ProjectileModel(const std::string& name)
: name_(name), damage_(10), splash_radius_(1.0), armor_piercing_(1.0), shot_cost_(10), lifetime_(30), 
mass_(10), mesh_name_(""), linear_speed_(50), angular_speed_(5), motion_lifetime_(2.0),
min_bonus_damage_(1), max_bonus_damage_(10), min_bonus_speed_(25), max_bonus_speed_(110)
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

void ProjectileModel::OnFire(const Target& target) {

}
void ProjectileModel::OnHit(Projectile& self, Ship& target, const std::vector<ugdk::action::mode3d::component::ContactPoint>& pts) {

}

double ProjectileModel::GetBonusDamage(double speed) const {
    /*
     if speed = X, speed_range(min,max) = [A, B], damage_range(min,max) = [C, D], then this function returns V so that:
     (V-C)/(D-C) = (X-A)/(B-A)
       <=>
     V = C + (D-C)(X-A)/(B-A)
    */
    if (speed < min_bonus_speed_) return min_bonus_damage_;
    if (speed > max_bonus_speed_) return max_bonus_damage_;
    double speed_range = max_bonus_speed_ - min_bonus_speed_;
    if (speed_range <= 0.0) return min_bonus_damage_;
    double speed_amount = speed - min_bonus_speed_;
    double dmg_range = max_bonus_damage_ - min_bonus_damage_;
    return min_bonus_damage_ + (dmg_range * speed_amount / speed_range);
}

} // namespace objects
} // namespace shipsbattle
