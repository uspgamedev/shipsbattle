#ifndef SHIPSBATTLE_COMPONENTS_SPACEDUST_H
#define SHIPSBATTLE_COMPONENTS_SPACEDUST_H

#include <ugdk/action/3D/component.h>

#include <OgreVector3.h>
#include <string>

namespace shipsbattle {
namespace components {
namespace subsystems {

class ShipSubsystem : public ugdk::action::mode3d::Component {

public:
    /// Name of the subsystem.
    std::string name() const { return name_; }
    /// Hitpoints (life) of this subsystem.
    double hitpoints() const { return hitpoints_; }
    void set_hitpoints(double hp) { hitpoints_ = hp; }
    /// If this subsystem is required by the ship. When a required subsystem is destroyed, the ship is destroyed.
    bool required() const { return required_; }
    void set_required(bool is_required) { required_ = is_required; }

    /// Position of the subsystem in the ship.
    Ogre::Vector3 position() const { return position_; }
    void set_position(const Ogre::Vector3& pos);
    /// Radius of the subsystem
    double radius() const { return radius_; }
    void set_radius(double rad) { radius_ = rad; }

protected:
    ShipSubsystem(const std::string& name) : name_(name) {}

    std::string  name_;
    double hitpoints_;
    bool required_;
    Ogre::Vector3 position_;
    double radius_;
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_SPACEDUST_H
