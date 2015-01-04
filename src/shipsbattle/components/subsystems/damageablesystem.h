#ifndef SHIPSBATTLE_SUBSYSTEMS_DAMAGEABLESYSTEM_H
#define SHIPSBATTLE_SUBSYSTEMS_DAMAGEABLESYSTEM_H

#include <string>
#include <memory>

class btVector3;
class btCollisionObject;
class btSphereShape;

namespace ugdk {
namespace action {
namespace mode3d {
class Component;
}}}

namespace shipsbattle {
namespace components {
namespace subsystems {

class DamageableSystem {

public:
    virtual ~DamageableSystem();

    /** Name of the subsystem. */
    std::string name() const { return name_; }

    /** Maximum Hitpoints (max life) of this subsystem.*/
    double max_hitpoints() const { return max_hitpoints_; }
    void set_max_hitpoints(double max_hp) { max_hitpoints_ = max_hp; }
    /** Hitpoints (life) of this subsystem.*/
    double hitpoints() const { return hitpoints_; }
    void set_hitpoints(double hp) { hitpoints_ = hp; }
    /** Maximum Armor rating of this hull. More armor means less damage taken.*/
    double max_armor_rating() const { return max_armor_rating_; }
    void set_max_armor_rating(double armor) { max_armor_rating_ = armor; }
    /** Armor rating of this hull. More armor means less damage taken.*/
    double armor_rating() const { return armor_rating_; }
    void set_armor_rating(double armor) { armor_rating_ = armor; }
    /** If this subsystem is required by the ship. When a required subsystem is destroyed, the ship is destroyed.*/
    bool required() const { return required_; }
    void set_required(bool is_required) { required_ = is_required; }
    /** Armor rating of this hull. More armor means less damage taken.*/
    double disabled_percentage() const { return disabled_percentage_; }
    void set_disabled_percentage(double dis_pc) { disabled_percentage_ = dis_pc; }
    /** If this subsystem is destroyed.*/
    bool destroyed() const { return hitpoints_ <= 0.0; }
    /** If this subsystem is disabled.*/
    bool disabled() const { return hitpoints_ <= (max_hitpoints_ * disabled_percentage_); }

    /** Sets the collision shape of this subsystem.*/
    void SetVolume(double radius, const btVector3& pos);
    btCollisionObject* volume() const { return volume_; }
    /** Local Position of the subsystem in the ship.*/
    btVector3 position() const;
    /** Absolute position of the subsystem in the world. */
    btVector3 world_position() const;
    /** Radius of the subsystem*/
    double radius() const;

    /** Gets the parent component system of this subsystems. */
    ugdk::action::mode3d::Component* parent() const { return parent_; }

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
    double disabled_percentage_;

    btCollisionObject* volume_;
    btSphereShape* shape_;

    ugdk::action::mode3d::Component* parent_;
    void RegisteredTo(ugdk::action::mode3d::Component* sys);
    virtual void OnRegister() {}
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_DAMAGEABLESYSTEM_H
