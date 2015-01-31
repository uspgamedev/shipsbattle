#ifndef SHIPSBATTLE_SUBSYSTEMS_THRUSTER_H
#define SHIPSBATTLE_SUBSYSTEMS_THRUSTER_H

#include <shipsbattle/components/subsystems/poweredsystem.h>
#include <OgreVector3.h>

namespace shipsbattle {
namespace components {
class Motion;

namespace subsystems {

class Thruster : public PoweredSystem {
public:
    Thruster(const std::string& name);

    virtual double NeedsRecharge() override;
    virtual void OnRecharge(double energy) override;

    /** Direction to where this thruster is pointing. Therefore the force it applies to 
    the ship is contrary to this. */
    Ogre::Vector3 thrust_direction() const { return thrust_direction_; }
    void set_thrust_direction(const Ogre::Vector3& dir) { thrust_direction_ = dir.normalisedCopy(); }
    /** Amount of thrust this Thruster can output per second at regular power levels and efficiency.
     Unit is WAT WAT WAT */
    double thrust_power() const { return thrust_power_; }
    void set_thrust_power(double power) { thrust_power_ = power; }

    /** Gets the current amount of thrust the engine can output, according to power levels and efficiency. */
    double current_thrust_power() const;

protected:
    // thrust directions
    Ogre::Vector3 thrust_direction_;
    // thrust power/force
    double thrust_power_;

    double spent_energy_;

    friend class Motion;
    // Receives thrust power (not percentage).
    void GenerateThrust(double power, double dt);
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_THRUSTER_H
