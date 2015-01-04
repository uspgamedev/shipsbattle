#ifndef SHIPSBATTLE_OBJECTS_PROJECTILEMODEL_H_
#define SHIPSBATTLE_OBJECTS_PROJECTILEMODEL_H_

#include <shipsbattle/components/subsystems/typedefs.h>

namespace shipsbattle {
namespace components {
namespace subsystems {
class DamageableSystem;
}
}
namespace objects {

class ProjectileModel {
public:
    ProjectileModel(const std::string& name);

    void OnFire(components::subsystems::DamageableSystem* target);
    void OnHit();

    /** Name of this projectile model. */
    std::string name() const { return name_; }
    /** Regular damage done by this weapon on hit. */
    double damage() const { return damage_; }
    void set_damage(double dmg) { damage_ = dmg; }
    /** Radius of explosion/damage on hit of this weapon. */
    double splash_radius() const { return splash_radius_; }
    void set_splash_radius(double radius) { splash_radius_ = radius; }
    /** Percentage of armor piercing of this weapon (between [0,1]). */
    double armor_piercing() const { return armor_piercing_; }
    void set_armor_piercing(double pierce);
    /** Function to modify damage on hit based on distance to point of impact. Receives the distance and
    splash_radius, should return value to multiply damage with. */
    const components::subsystems::DecaymentFunction& decayment() const { return decayment_; }
    void set_decayment(const components::subsystems::DecaymentFunction& dec_func) { decayment_ = dec_func; }
    /** Energy cost of each shot. */
    double shot_cost() const { return shot_cost_; }
    void set_shot_cost(double cost) { shot_cost_ = cost; }
    /** Time (in seconds) this projectile will exist before detonating prematurely. */
    double lifetime() const { return lifetime_; }
    void set_lifetime(double time) { lifetime_ = time; }
    /** Mass of the projectile, in kg. */
    double mass() const { return mass_; }
    void set_mass(double m) { mass_ = m; }
    /** Name of mesh to use for this projectile. */
    std::string mesh_name() const { return mesh_name_; }
    void set_mesh_name(const std::string& mname) { mesh_name_ = mname; }
    /** Linear speed of this projectile, while cruising. */
    double linear_speed() const { return linear_speed_; }
    void set_linear_speed(double lin_spd) { linear_speed_ = lin_spd; }
    /** Angular speed of this projectile, while cruising to track its target. */
    double angular_speed() const { return angular_speed_; }
    void set_angular_speed(double ang_spd) { angular_speed_ = ang_spd; }

protected:
    std::string name_;
    // hit/damage attributes
    double damage_;
    double splash_radius_;
    double armor_piercing_;
    components::subsystems::DecaymentFunction decayment_;
    // life attributes
    double shot_cost_;
    double lifetime_;
    double mass_;
    std::string mesh_name_;
    // motion attributes
    double linear_speed_;
    double angular_speed_;
};

} // namespace objects
} // namespace shipsbattle
#endif //SHIPSBATTLE_OBJECTS_PROJECTILEMODEL_H_
