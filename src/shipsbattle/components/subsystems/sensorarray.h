#ifndef SHIPSBATTLE_SUBSYSTEMS_SENSORARRAY_H
#define SHIPSBATTLE_SUBSYSTEMS_SENSORARRAY_H

#include <shipsbattle/components/subsystems/poweredsystem.h>

namespace shipsbattle {
namespace components {
class Navigation;

namespace subsystems {

class SensorArray : public PoweredSystem {
public:
    SensorArray(const std::string& name);

    virtual double NeedsRecharge() override;
    virtual void OnRecharge(double energy) override;

    /** Refresh rate of the sensor, in seconds. That is, time interval between sensor sweeps. */
    double refresh_rate() const { return refresh_rate_; }
    void set_refresh_rate(double rate) { refresh_rate_ = rate; }
    /** Maximum range (in game units) the sensor can detect objects. */
    double maximum_range() const { return maximum_range_; }
    void set_maximum_range(double range) { maximum_range_ = range; }

protected:
    double elapsed_;
    double refresh_rate_;
    double maximum_range_;

    friend class Navigation;
    virtual void Update(double dt);
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_SENSORARRAY_H
