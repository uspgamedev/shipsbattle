#ifndef SHIPSBATTLE_SUBSYSTEMS_IMPULSEENGINE_H
#define SHIPSBATTLE_SUBSYSTEMS_IMPULSEENGINE_H

#include <shipsbattle/components/subsystems/poweredsystem.h>
#include <OgreVector3.h>

namespace shipsbattle {
namespace components {
class Motion;

namespace subsystems {

class ImpulseEngine : public PoweredSystem {
public:
    ImpulseEngine(const std::string& name);

    virtual double NeedsRecharge() override;
    virtual void OnRecharge(double energy) override;

    /** Default direction the engine is pointing at. */
    Ogre::Vector3 exhaust_direction() const { return exhaust_direction_; }
    void set_exhaust_direction(const Ogre::Vector3& dir) { exhaust_direction_ = dir; }
    /** Angle to the exhaust direction in which the engine can point at, in radians. 
    This forms a cone in which the engine can direct its exhaust. */
    double exhaust_angle() const { return exhaust_angle_; }
    void set_exhaust_angle(double angle) { exhaust_angle_ = angle; }
    /** Amount of thrust the engine can output at regular power levels and efficiency. */
    double exhaust_power() const { return exhaust_power_; }
    void set_exhaust_power(double power) { exhaust_power_ = power; }

protected:
    // exhaust directions
    Ogre::Vector3 exhaust_direction_;
    double exhaust_angle_;
    // exhaust power/force
    double exhaust_power_;

    friend class Motion;
    virtual void Update(double dt);
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_IMPULSEENGINE_H
