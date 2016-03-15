#ifndef SHIPSBATTLE_COMPONENTS_MOTION_H
#define SHIPSBATTLE_COMPONENTS_MOTION_H

#include <shipsbattle/components/updateablecomponent.h>

#include <Eigen/Core>
#include <OgreVector3.h>

#include <vector>
#include <unordered_map>

namespace Ogre {
class ParticleSystem;
class ParticleEmitter;
}

namespace ugdk {
namespace action {
namespace mode3d {
namespace component {
class Body;
}}}}

namespace shipsbattle {
namespace components {
namespace subsystems {
class ImpulseEngine;
class Thruster;
}

class Motion : public UpdateableComponent {
public:
    Motion();
    virtual ~Motion();

    virtual std::type_index type() const override;

    void AddImpulseEngine(const std::shared_ptr<subsystems::ImpulseEngine>& engine);
    const std::shared_ptr<subsystems::ImpulseEngine>& GetImpulseEngine(size_t index);
    const std::shared_ptr<subsystems::ImpulseEngine>& GetImpulseEngine(const std::string& name);
    size_t GetNumImpulseEngines() const { return impulse_engines_.size(); }

    void AddThruster(const std::shared_ptr<subsystems::Thruster>& thruster);
    const std::shared_ptr<subsystems::Thruster>& GetThruster(size_t index);
    const std::shared_ptr<subsystems::Thruster>& GetThruster(const std::string& name);
    size_t GetNumThrusters() const { return thrusters_.size(); }

    void Update(double dt) override;

    /** Makes the object move towards desired direction with desired power, if possible, for duration. If dir or power are zero,
    then the resulting motion will be one to stop the object's movement. This will cancel previous move commands.
    @param dir Direction to move towards, should be relative to the object. For example, to move "forward", dir=UnitZ.
    @param power Amount of power to use in movement. Should be in the range [0, 1], with 0 being 'no power/stop' and 1 being full power. 
        The actual power the engines will use depend on their integrity and power levels, but power 1 here will try to use the current full power.
    @param duration Time, in seconds, which the engines shall maintain this motion. */
    void MoveTowards(const Ogre::Vector3& dir, double power, double duration);

    /** Returns if the engines are currently performing any movement commands. */
    bool IsMoving() const;

    /** Cancels current movement orders. Possibly also adds an order to stop motion, that is, to counteract current
    linear velocity in order to bring the object to a halt. */
    void StopMoving();

    /** Makes the object turn towards desired direction with desired power, if possible, for duration. If dir or power are zero,
    then the resulting rotation will be one to stop the object's rotation. This will cancel previous turn commands.
    @param dir Direction to turn (point) towards, should be relative to the object. For example, since forward is UnitZ, turning to UnitZ will cause nothing.
    @param power Amount of power to use in turning. Should be in the range [0, 1], with 0 being 'no power/stop', and 1 being full power.
        The actual power the thrusters will use depend on their integrity and power levels, but power 1 here will try to use the current full power.
    @param duration Time, in seconds, which the thrusters shall maintain this rotation. */
    void TurnTowards(const Ogre::Vector3& dir, double power, double duration);
    /** Makes the object turn (rotate) around the desired axis with desired power, if possible, for duration. If axis or power are zero,
    then the resulting rotation will be one to stop the object's rotation. This will cancel previous turn commands.
    @param axis Axis to rotate around, relative to the object. Since this method will always turn to one way, to turn to the other axis should be reversed
        (user can do this manually or pass a negative power value).
    @param power Amount of power to use in turning. Should be in the range [-1, 1], with 0 being 'no power/stop', 1 being full power clockwise around the axis,
        and -1 being full power counterclockwise (reversed axis). */
    void TurnAround(const Ogre::Vector3& axis, double power, double duration);

    /** Returns if the thrusters are currently performing any turning commands. */
    bool IsTurning() const;

    /** Cancels current turning orders. Possibly also adds an order to stop rotation, that is, to conteract current
    angular velocity in order to bring the object to a halt (rotation wise). */
    void StopTurning();

    /** Stops all movement, stopping current movement and turning orders. */
    void AllStop();

    /** If we are actively using our thrusters to counteract any torque(rotation) inflicted by movement. */
    bool cancel_move_torque() const { cancel_move_torque_; }
    void set_cancel_move_torque(bool cancel) { cancel_move_torque_ = cancel; }
    /** Speed (magnitude), in GameUnits/s, threshold to assume a linear velocity vector is null (zero). */
    double move_stop_threshold() const { return move_stop_threshold_; }
    void set_move_stop_threshold(double threshold) { move_stop_threshold_ = threshold; }
    /** Angular speed (magnitude), in Radians/s, threshold to assume a angular velocity vector is null (zero). */
    double turn_stop_threshold() const { return turn_stop_threshold_; }
    void set_turn_stop_threshold(double threshold) { turn_stop_threshold_ = threshold; }
    /** Angle threshold, in radians, to assume 2 direction vectors are the same (pointing the same way). */
    double angle_threshold() const { return angle_threshold_; }
    void set_angle_threshold(double threshold) { 
        angle_threshold_ = threshold; turn_.angle_threshold = threshold; move_.angle_threshold = threshold;
    }


protected:
    std::vector<std::shared_ptr<subsystems::ImpulseEngine>>    impulse_engines_;
    std::unordered_map<std::string, size_t>    impulse_indexes_;

    std::vector<std::shared_ptr<subsystems::Thruster>>    thrusters_;
    std::unordered_map<std::string, size_t>    thruster_indexes_;

    enum MotionStatus { ACTIVE, STOPPING, STOPPED };
    struct MotionData {
        Ogre::Vector3 direction;
        double power;
        double duration;
        double elapsed;
        MotionStatus status;
        bool update_system;
        double angle_threshold;
        Eigen::VectorXd factors;
        Eigen::VectorXd outputs;

        MotionData();
        void Reset(const Ogre::Vector3& dir, double p, double dur, MotionStatus stat);
    };
    MotionData move_;
    MotionData turn_;
    bool cancel_move_torque_;
    double move_stop_threshold_; //GU/s threshold to force 0 linear velocity.
    double turn_stop_threshold_; //radians/s threshold to force 0 angular velocity.
    double angle_threshold_; //angle (radians) threshold to assume 2 vectors are pointing the same way.
    size_t last_active_engines_; // number of engines that were active
    size_t last_active_thrusters_;
    Ogre::Vector3 generated_torque_;

    ugdk::action::mode3d::component::Body* parent_body_;

    /** dir is the target IMPULSE direction in local coords, therefore it is -(target movement direction) */
    void DoMove(const Ogre::Vector3& dir, double power, double dt);
    /** axis is the target THRUST axis (direction) in local coords, therefore it is -(target rotation axis) */
    void DoTurn(const Ogre::Vector3& axis, double power, double dt);
    void ForceMoveStop();
    void ForceTurnStop();
    struct MotionSystemPair {
        Ogre::Vector3 vector;
        double output;

        MotionSystemPair() {}
        MotionSystemPair(const Ogre::Vector3& v, double out) : vector(v), output(out) {}
    };
    bool ResolveMotionSystem(MotionData& data, const Ogre::Vector3& target, const std::vector<MotionSystemPair>& values, size_t& last_active);
    void CalculateOutputValues(Eigen::VectorXd& outputs, const Eigen::VectorXd& factors);

    void OnTaken() override;

    struct ParticleEffect {
        Ogre::ParticleSystem* system;
        std::vector<Ogre::ParticleEmitter*> emitters;
        double emission_rate;
        double time_to_live;
        double velocity_min;
        double velocity_max;
    };
    ParticleEffect impulse_effects_;
    ParticleEffect thruster_effects_;
    void AddEmitter(const Ogre::Vector3& pos, const Ogre::Vector3& dir, double radius, ParticleEffect& effect);
    void EnableEmitter(const ParticleEffect& effect, int emitter_index, double intensity);
};

inline std::type_index Motion::type() const {
    return typeid(Motion);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_MOTION_H
