#ifndef SHIPSBATTLE_SUBSYSTEMS_DAMAGEABLESYSTEM_H
#define SHIPSBATTLE_SUBSYSTEMS_DAMAGEABLESYSTEM_H

#include <OgreVector3.h>
#include <string>

namespace shipsbattle {
namespace components {
class Hull;

namespace subsystems {

class DamageableSystem {

public:
    /// Name of the subsystem.
    std::string name() const { return name_; }

    /// Maximum Hitpoints (max life) of this subsystem.
    double max_hitpoints() const { return max_hitpoints_; }
    void set_max_hitpoints(double max_hp) { max_hitpoints_ = max_hp; }
    /// Hitpoints (life) of this subsystem.
    double hitpoints() const { return hitpoints_; }
    void set_hitpoints(double hp) { hitpoints_ = hp; }
    /// Maximum Armor rating of this hull. More armor means less damage taken.
    double max_armor_rating() const { return max_armor_rating_; }
    void set_max_armor_rating(double armor) { max_armor_rating_ = armor; }
    /// Armor rating of this hull. More armor means less damage taken.
    double armor_rating() const { return armor_rating_; }
    void set_armor_rating(double armor) { armor_rating_ = armor; }
    /// If this subsystem is required by the ship. When a required subsystem is destroyed, the ship is destroyed.
    bool required() const { return required_; }
    void set_required(bool is_required) { required_ = is_required; }
    /// If this subsystem is destroyed.
    bool destroyed() const { return hitpoints_ <= 0.0; }

    /// Position of the subsystem in the ship.
    Ogre::Vector3 position() const { return position_; }
    void set_position(const Ogre::Vector3& pos);
    /// Radius of the subsystem
    double radius() const { return radius_; }
    void set_radius(double rad) { radius_ = rad; }

    /** Method to deal/fix damage to this subsystems, updating the hitpoints and armor rating 
    of this ship by a given amount. Positive amounts deal damage, negative amounts fix the system.
    Amount passed should be the raw damage sustained, and the piercing porcentage (0-1).
    This method will internally calculate based on armor_rating how much actual damage it'll take.
    If hitpoints drop below zero, it will notify parent system/ship of its destruction.
    */
    void TakeDamage(double dmg, double piercing);

protected:
    DamageableSystem(const std::string& name);

    std::string  name_;
    double max_hitpoints_;
    double hitpoints_;
    double max_armor_rating_;
    double armor_rating_;
    bool required_;
    Ogre::Vector3 position_;
    double radius_;

    void RegisterToHull(Hull* hull);
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_DAMAGEABLESYSTEM_H
