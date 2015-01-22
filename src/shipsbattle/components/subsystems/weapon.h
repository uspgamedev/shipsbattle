#ifndef SHIPSBATTLE_SUBSYSTEMS_WEAPON_H
#define SHIPSBATTLE_SUBSYSTEMS_WEAPON_H

#include <shipsbattle/components/subsystems/poweredsystem.h>
#include <OgreVector3.h>

namespace shipsbattle {
namespace objects {
class Target;
}
namespace components {
class Tactical;

namespace subsystems {
class DamageableSystem;

class Weapon : public PoweredSystem {
public:
    virtual double NeedsRecharge() override;
    virtual void OnRecharge(double energy) override;

    bool CanFire() const;
    virtual bool CanFireAt(const objects::Target& target);
    bool TryFire(const objects::Target& target);
    virtual bool Fire(const objects::Target& target) = 0;

    /** Return time left to be able to fire again. */
    double elapsed() const { return elapsed_; }
    /** Minimum time (in seconds) between shots of this weapon. */
    double cooldown() const { return cooldown_; }
    void set_cooldown(double cldwn) { cooldown_ = cldwn; }
    /** Current amount of energy stored in this weapon. */
    double energy() const { return energy_; }
    void set_energy(double enrgy);
    /** Maximum amount of energy this weapon can store. */
    double max_energy() const { return max_energy_; }
    void set_max_energy(double max_enrgy);
    /** Gets the base direction this weapon is pointing at (should be away from ship mesh) */
    Ogre::Vector3 direction() const { return direction_; }
    void set_direction(const Ogre::Vector3& dir) { direction_ = dir; }
    /** Gets the angle, in degrees, from the direction in which the weapon can fire. This forms a conical area-of-fire. */
    double firing_angle() const { return firing_angle_; }
    void set_firing_angle(double angle) { firing_angle_ = angle; }

protected:
    Weapon(const std::string& name);

    double elapsed_;
    double cooldown_;
    double energy_;
    double max_energy_;
    Ogre::Vector3 direction_;
    double firing_angle_;

    friend class Tactical;
    virtual void Update(double dt);
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_WEAPON_H
