#include <shipsbattle/components/motion.h>
#include <shipsbattle/components/subsystems/impulseengine.h>
#include <shipsbattle/components/subsystems/thruster.h>
#include <shipsbattle/utils/matrix.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/debug/log.h>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using ugdk::action::mode3d::component::Body;
using shipsbattle::components::subsystems::ImpulseEngine;
using shipsbattle::components::subsystems::Thruster;
using shipsbattle::utils::Matrix;
using Ogre::Vector3;

namespace shipsbattle {
namespace components {

Motion::MotionData::MotionData()
: direction(Vector3::UNIT_Z), power(0.0), duration(0.0), elapsed(0.0), status(STOPPED)
{

}

void Motion::MotionData::Reset(const Ogre::Vector3& dir, double p, double dur, MotionStatus stat) {
    direction = dir;
    power = p;
    duration = dur;
    elapsed = 0.0;
    status = stat;
    factors = Eigen::VectorXd();
}

Motion::Motion() : move_stop_threshold_(0.001), angle_threshold_(0.01), last_active_engines_(0) {
}

void Motion::AddImpulseEngine(const shared_ptr<ImpulseEngine>& engine) {
    if (impulse_indexes_.count(engine->name()) > 0) {
        Log(LogLevel::WARNING, "Motion System", "ImpulseEngine with name '" + engine->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Impulse Engine.");
        return;
    }
    impulse_engines_.push_back(engine);
    impulse_indexes_[engine->name()] = impulse_engines_.size() - 1;
    engine->RegisteredTo(this);
}
const shared_ptr<ImpulseEngine>& Motion::GetImpulseEngine(size_t index) {
    return impulse_engines_.at(index);
}
const shared_ptr<ImpulseEngine>& Motion::GetImpulseEngine(const std::string& name) {
    return GetImpulseEngine(impulse_indexes_[name]);
}

void Motion::AddThruster(const shared_ptr<Thruster>& thruster) {
    if (thruster_indexes_.count(thruster->name()) > 0) {
        Log(LogLevel::WARNING, "Motion System", "Thruster with name '" + thruster->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Thruster.");
        return;
    }
    thrusters_.push_back(thruster);
    thruster_indexes_[thruster->name()] = thrusters_.size() - 1;
    thruster->RegisteredTo(this);
}
const shared_ptr<Thruster>& Motion::GetThruster(size_t index) {
    return thrusters_.at(index);
}
const shared_ptr<Thruster>& Motion::GetThruster(const std::string& name) {
    return GetThruster(thruster_indexes_[name]);
}

void Motion::Update(double dt) {
    // if stopped, check if we're moving - if so, we need to stop.
    auto body = owner()->component<Body>();
    if (move_.status == STOPPED) {
        if (body->linear_velocity().length() > move_stop_threshold_) {
            move_.status = STOPPING;
        }
    }

    // MOVE
    if (move_.status == ACTIVE) {
        if (move_.elapsed <= move_.duration) {
            move_.elapsed += dt;
            DoMove(move_.direction, move_.power, dt);
        }
        else {
            move_.status = STOPPED;
        }
    }
    else if (move_.status == STOPPING) {
        DoMove(body->linear_velocity()*-1, move_.power, dt);
        if (body->linear_velocity().length() <= move_stop_threshold_) {
            move_.status = STOPPED;
            body->set_linear_velocity(Ogre::Vector3::ZERO);
        }
    }

    // TURN
}

void Motion::MoveTowards(const Ogre::Vector3& dir, double power, double duration) {
    Ogre::Vector3 direction = dir;
    MotionStatus stat = ACTIVE;
    auto len = dir.length();
    if (len == 0.0 || power <= 0.0) {
        power = 1.0;
        direction = owner()->component<Body>()->linear_velocity() * -1;
        direction.normalise();
        stat = STOPPING;
    }
    else if (len != 1.0) {
        direction.normalise();
    }

    if (power < 0.0) power = 0.0;
    else if (power > 1.0) power = 1.0;

    move_.Reset(direction, power, duration, stat);
}

bool Motion::IsMoving() const {
    return move_.status != STOPPED;
}

void Motion::StopMoving() {

}

void Motion::TurnTowards(const Ogre::Vector3& dir, double power, double duration) {

}

bool Motion::IsTurning() const {
    return turn_.status != STOPPED;
}

void Motion::StopTurning() {

}

void Motion::AllStop() {
    StopMoving();
    StopTurning();
}

void Motion::DoMove(const Ogre::Vector3& dir, double power, double dt) {
    std::vector<std::shared_ptr<subsystems::ImpulseEngine>> active_engines;
    for (auto engine : impulse_engines_) {
        if (!engine->activated() || engine->disabled()) continue;
        active_engines.push_back(engine);
    }
    // this function has several FOR loops over active_engines.
    // unfortunately, at the point of this writing I could not see a way around this
    // since the work done in one requires that the previous one has already been executed

    if ((dir.angleBetween(move_.direction).valueRadians() > angle_threshold_) || (last_active_engines_ != active_engines.size())) {
        last_active_engines_ = active_engines.size();
        if (active_engines.size() <= 0) return;

        //get list of active engines, calculate engine factors and store maximal outputs.
        Matrix dirs = Matrix(3, active_engines.size());
        move_.outputs.resize(active_engines.size());
        for (int i = 0; i < active_engines.size(); i++) {
            auto engine = active_engines[i];
            auto eng_dir = engine->GetVectorClosestTo(dir);
            eng_dir.normalise();
            dirs.col(i) = Eigen::Vector3d(eng_dir.x, eng_dir.y, eng_dir.z);
            move_.outputs(i) = engine->current_exhaust_power();
            engine->set_current_direction(eng_dir);
        }
        /* for each engine i, we have its vector Vi: closest vector it can point at towards
        target direction D.
        In order to properly generate thrust towards D, we have to balance the power on each
        engine towards Vi properly. Therefore we need to find Fi for each engine i so that:

        sum[Vi*Fi] = V1*F1 + V2*F2 + ... + Vn*Fn = D
        Since Vi's and D are vectors, and Fi's are scalars, we can rewrite this equation as:

        V * F = D
        where V is a 3xN matrix (each column is a Vi), F is a vector (size N) of Fi's, and D is itself (size 3 vector)
        We have V and D and need to compute F. Therefore:
        F = inverseV * D
        Where inverseV is the pseudoinverse matrix of V (since V can be non-square).
        */
        move_.factors = dirs.pseudoInverse() * Eigen::Vector3d(dir.x, dir.y, dir.z);

        // normalize factors and remove negative coefficients
        if (move_.factors.maxCoeff() <= 0.0)    return;
        move_.factors /= (1.0 / move_.factors.maxCoeff());
        double reference_output = 0.0;
        for (int i = 0; i < active_engines.size(); i++) {
            if (move_.factors(i) < 0.0)   move_.factors(i) = 0.0;
        }
    }

    // calculate actual engine outputs.
    //   doing this here prevents us from having to add extra logic to handle other cases in which 
    //   we would need to recalculate engines output.
    move_.outputs.resize(active_engines.size());
    for (int i = 0; i < active_engines.size(); i++) {
        move_.outputs(i) = active_engines[i]->current_exhaust_power();
    }
    CalculateEnginesOutput();

    // do engine update / apply impulse
    for (int i = 0; i < active_engines.size(); i++) {
        active_engines[i]->GenerateThrust(move_.outputs(i) * power, dt);
        //TODO: collect generated torque
    }
}

void Motion::CalculateEnginesOutput() {
    /* For each engine (index) i, we have:
        0 <= Pi <= Mi, where Mi is the maximal engine output, and Pi is the adjusted output
                       which should be used now.
        also,
        Pi/Pj = Fi/Fj, for every engine index j, where Fi = move_.factors(i) dictates the power
                        relationship between engines.
        therefore:
        Pi = Pj*Fi/Fj <= Mj*Fi/Fj, for every j

        so firstly we calculate all upper bounds based on Mi, and get the minimum of them
        to get an initial Pi, taking care for possible Fi's that are 0.
        The minimum of the upper bounds is required because it is the only one that validates
        all Pi <= Mi inequalities.
    */
    int i;
    for (i = 0; i < move_.factors.size(); i++) if (move_.factors(i) > 0.0)  break;
    for (int j = 0; j < move_.outputs.size(); j++) {
        if (move_.factors(j) > 0.0)
            move_.outputs(j) = move_.outputs(j) * move_.factors(i) / move_.factors(j);
    }
    double Pi = move_.outputs.minCoeff();

    // With an initial Pi, we can simply use the equality formula to calculate all Pi's
    for (int j = 0; j < move_.outputs.size(); j++) {
        move_.outputs(j) = Pi * move_.factors(j) / move_.factors(i);
    }
}

} // namespace components
} // namespace shipsbattle
