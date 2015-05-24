#include <shipsbattle/components/motion.h>
#include <shipsbattle/components/subsystems/impulseengine.h>
#include <shipsbattle/components/subsystems/thruster.h>
#include <shipsbattle/utils/matrix.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/debug/log.h>

#include <BtOgreExtras.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreParticleSystem.h>
#include <OgreParticleEmitter.h>

#include <iostream>

using std::cout;
using std::endl;
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
: direction(Vector3::UNIT_Z), power(0.0), duration(0.0), elapsed(0.0), status(STOPPED), update_system(false), angle_threshold(0.01)
{

}

void Motion::MotionData::Reset(const Ogre::Vector3& dir, double p, double dur, MotionStatus stat) {
    update_system = (dir.angleBetween(direction).valueRadians() > angle_threshold);
    direction = dir;
    power = p;
    duration = dur;
    elapsed = 0.0;
    status = stat;
}

Motion::Motion() : move_stop_threshold_(0.001), turn_stop_threshold_(0.01), angle_threshold_(0.01), last_active_engines_(-1),
last_active_thrusters_(-1), generated_torque_(Ogre::Vector3::ZERO) {
    impulse_effects_.system = nullptr;
    thruster_effects_.system = nullptr;
}

Motion::~Motion() {
    //TODO: we need to properly delete these particle systems here,
    //      in case we are being deleted in the middle of the scene.
    //      No need to delete along with scene deletion since Scene3D deletion will
    //      remove all particle systems.
    //owner()->scene().manager()->destroyParticleSystem(jet_system_);
    //owner()->scene().manager()->destroyParticleSystem(smoke_system_);
}

void Motion::AddImpulseEngine(const shared_ptr<ImpulseEngine>& engine) {
    if (impulse_indexes_.count(engine->name()) > 0) {
        Log(LogLevel::WARNING, "Motion System", "ImpulseEngine with name '" + engine->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this Impulse Engine.");
        return;
    }
    impulse_engines_.push_back(engine);
    impulse_indexes_[engine->name()] = impulse_engines_.size() - 1;
    engine->RegisteredTo(this);
    AddEmitter(BtOgre::Convert::toOgre(engine->position()), engine->exhaust_direction(), engine->radius(), impulse_effects_);
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
    AddEmitter(BtOgre::Convert::toOgre(thruster->position()), thruster->thrust_direction(), thruster->radius(), thruster_effects_);
}
const shared_ptr<Thruster>& Motion::GetThruster(size_t index) {
    return thrusters_.at(index);
}
const shared_ptr<Thruster>& Motion::GetThruster(const std::string& name) {
    return GetThruster(thruster_indexes_[name]);
}

void Motion::Update(double dt) {
    // if stopped, check if we're moving - if so, we need to stop.
    if (move_.status == STOPPED) {
        if (parent_body_->linear_velocity().length() > move_stop_threshold_) {
            StopMoving();
        }
    }
    if (turn_.status == STOPPED) {
        if (parent_body_->angular_velocity().length() > turn_stop_threshold_) {
            StopTurning();
        }
    }

    if (owner()->name() == "Player") {
        static int count = 0;
        if (count >= 150) {
            cout << "Turn=" << turn_.status << " AngVel= " << parent_body_->angular_velocity().x;
            cout << ", " << parent_body_->angular_velocity().y << ", " << parent_body_->angular_velocity().z << endl;
            count = 0;
        }
        count++;
    }

    // MOVE
    generated_torque_ *= 0.0;
    if (move_.status == ACTIVE) {
        if (move_.elapsed <= move_.duration) {
            move_.elapsed += dt;
            DoMove(move_.direction, move_.power, dt);
        }
        else {
            ForceMoveStop();
        }
    }
    else if (move_.status == STOPPING) {
        auto stop_dir = parent_body_->linear_velocity();
        stop_dir.normalise();
        DoMove(stop_dir, move_.power, dt);
        auto lin_vel = parent_body_->linear_velocity();
        lin_vel.normalise();
        if (parent_body_->linear_velocity().length() <= move_stop_threshold_ || lin_vel.dotProduct(stop_dir) <= 0.0) {
            ForceMoveStop();
        }
    }
    if (generated_torque_.length() > 0.0) {
        //TODO: we have generated torque and should try to cancel it. Update turn status.
    }

    // TURN
    if (turn_.status == ACTIVE) {
        if (turn_.elapsed <= turn_.duration) {
            turn_.elapsed += dt;
            DoTurn(turn_.direction - generated_torque_, turn_.power, dt);
        }
        else {
            ForceTurnStop();
        }
    }
    else if (turn_.status == STOPPING) {
        auto stop_dir = -(parent_body_->angular_velocity());
        stop_dir.normalise();
        DoTurn(stop_dir, turn_.power, dt);
        auto ang_vel = -(parent_body_->angular_velocity());
        ang_vel.normalise();
        if (parent_body_->angular_velocity().length() <= turn_stop_threshold_ || ang_vel.dotProduct(stop_dir) <= 0.0) {
            ForceTurnStop();
        }
    }
}

void Motion::MoveTowards(const Ogre::Vector3& dir, double power, double duration) {
    Ogre::Vector3 direction = parent_body_->orientation() * (-dir);
    //direction stored in move_ is the target ENGINE direction (resulting velocity will be to the other side)
    MotionStatus stat = ACTIVE;
    auto len = dir.length();
    if (len == 0.0 || power <= 0.0) {
        if (parent_body_->linear_velocity().length() <= move_stop_threshold_) {
            ForceMoveStop();
            return;
        }
        power = 1.0;
        direction = parent_body_->linear_velocity();
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
    MoveTowards(Ogre::Vector3::ZERO, 0.0, 0.0);
}

void Motion::TurnTowards(const Ogre::Vector3& dir, double power, double duration) {
    TurnAround(Ogre::Vector3::UNIT_Z.crossProduct(dir.normalisedCopy()), power, duration);
}
void Motion::TurnAround(const Ogre::Vector3& axis, double power, double duration) {
    Ogre::Vector3 ang_dir = parent_body_->orientation() * axis;
    MotionStatus stat = ACTIVE;
    auto len = ang_dir.length();
    if (len == 0.0 || power == 0.0) {
        if (parent_body_->angular_velocity().length() <= turn_stop_threshold_) {
            ForceTurnStop();
            return;
        }
        power = 1.0;
        ang_dir = parent_body_->angular_velocity() * -1;
        ang_dir.normalise();
        stat = STOPPING;
    }
    else if (len != 1.0) {
        ang_dir.normalise();
    }

    if (power < 0.0) {
        ang_dir *= -1;
        power = -power;
    }
    else if (power > 1.0) power = 1.0;

    turn_.Reset(ang_dir, power, duration, stat);
}

bool Motion::IsTurning() const {
    return turn_.status != STOPPED;
}

void Motion::StopTurning() {
    TurnAround(Ogre::Vector3::ZERO, 0.0, 0.0);
}

void Motion::AllStop() {
    StopMoving();
    StopTurning();
}

void Motion::DoMove(const Ogre::Vector3& dir, double power, double dt) {
    //dir is target engine direction
    std::vector<MotionSystemPair> values;
    for (int i = 0; i < impulse_engines_.size(); i++) {
        auto engine = impulse_engines_[i];
        auto emitter = impulse_effects_.emitters[i];
        emitter->setEnabled(false);
        if (!engine->activated() || engine->disabled()) {
            continue;
        }
        auto eng_dir = engine->GetVectorClosestTo(dir);
        values.push_back(MotionSystemPair(eng_dir, engine->current_exhaust_power()));
        engine->set_current_direction(eng_dir);
        emitter->setDirection(eng_dir);
    }
    
    if (!ResolveMotionSystem(move_, dir, values, last_active_engines_)) 
        return;

    // do engine update / apply impulse
    for (int i = 0; i < impulse_engines_.size(); i++) {
        auto engine = impulse_engines_[i];
        if (!engine->activated() || engine->disabled()) {
            continue;
        }
        double actual_power = move_.outputs(i) * power;
        if (actual_power > 0) {
            auto torque = engine->GenerateThrust(actual_power, dt);
            EnableEmitter(impulse_effects_, i, actual_power / engine->current_exhaust_power());
            if (cancel_move_torque_)
                generated_torque_ += torque;
        }
    }
}
void Motion::DoTurn(const Ogre::Vector3& axis, double power, double dt) {
    std::vector<MotionSystemPair> values;
    for (int i = 0; i < thrusters_.size(); i++) {
        auto thruster = thrusters_[i];
        auto emitter = thruster_effects_.emitters[i];
        emitter->setEnabled(false);
        if (!thruster->activated() || thruster->disabled()) {
            continue;
        }
        auto ang_dir = thruster->rotational_axis();
        values.push_back(MotionSystemPair(ang_dir, thruster->current_thrust_power()));
    }

    if (!ResolveMotionSystem(turn_, axis, values, last_active_thrusters_))
        return;

    // do engine update / apply impulse
    for (int i = 0; i < thrusters_.size(); i++) {
        auto thruster = thrusters_[i];
        if (!thruster->activated() || thruster->disabled()) {
            continue;
        }
        double actual_power = turn_.outputs(i) * power;
        if (actual_power > 0) {
            thruster->GenerateThrust(actual_power, dt);
            EnableEmitter(thruster_effects_, i, actual_power / thruster->current_thrust_power());
        }
    }
}

bool Motion::ResolveMotionSystem(MotionData& data, const Ogre::Vector3& target, const std::vector<MotionSystemPair>& values, size_t& last_active) {
    // this function has several FOR loops over active_engines.
    // unfortunately, at the point of this writing I could not see a way around this 
    // since the work done in one requires that the previous one has already been executed

    if ((target.angleBetween(data.direction).valueRadians() > angle_threshold_) || (last_active != values.size()) || data.update_system) {
        cout << "ResolveMotionSystem UPDATED FACTORS start. target: " << target.x << " " << target.y << " " << target.z << endl;
        last_active = values.size();
        data.update_system = false;
        if (values.size() <= 0) return false;
        //CHECK: we might need to reset data.direction with target here

        //get list of active engines, calculate engine factors and store maximal outputs.
        Matrix dirs = Matrix(3, values.size());
        for (int i = 0; i < values.size(); i++) {
            auto& vec = values[i].vector;
            dirs.col(i) = Eigen::Vector3d(vec.x, vec.y, vec.z);
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
        data.factors = dirs.pseudoInverse() * Eigen::Vector3d(target.x, target.y, target.z);

        // normalize factors and remove negative coefficients
        if (data.factors.maxCoeff() <= 0.0)    return false;
        data.factors /= data.factors.maxCoeff();
        cout << "ResolveMotionSystem UPDATED FACTORS FINAL: ";
        for (int i = 0; i < values.size(); i++) {
            if (data.factors(i) < 0.0)   data.factors(i) = 0.0;
            cout << data.factors(i) << " ";
        }
        cout << endl;
    }
    if (values.size() <= 0) return false;

    // calculate actual engine outputs.
    //   doing this here prevents us from having to add extra logic to handle other cases in which 
    //   we would need to recalculate engines output.
    data.outputs.resize(values.size());
    for (int i = 0; i < values.size(); i++) {
        data.outputs(i) = values[i].output;
    }
    CalculateOutputValues(data.outputs, data.factors);
    return true;
}

void Motion::CalculateOutputValues(Eigen::VectorXd& outputs, const Eigen::VectorXd& factors) {
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
    for (i = 0; i < factors.size(); i++) if (factors(i) > 0.0)  break;
    for (int j = 0; j < outputs.size(); j++) {
        if (factors(j) > 0.0)
            outputs(j) = outputs(j) * factors(i) / factors(j);
    }
    double Pi = outputs.minCoeff();

    // With an initial Pi, we can simply use the equality formula to calculate all Pi's
    for (int j = 0; j < outputs.size(); j++) {
        outputs(j) = Pi * factors(j) / factors(i);
    }
}

void Motion::ForceMoveStop() {
    move_.status = STOPPED;
    for (auto emitter : impulse_effects_.emitters)
        emitter->setEnabled(false);
    parent_body_->set_linear_velocity(Ogre::Vector3::ZERO);
}
void Motion::ForceTurnStop() {
    turn_.status = STOPPED;
    for (auto emitter : thruster_effects_.emitters)
        emitter->setEnabled(false);
    parent_body_->set_angular_velocity(Ogre::Vector3::ZERO);
}

void Motion::OnTaken() {
    UpdateableComponent::OnTaken();

    // create particle systems
    auto parent = owner();
    auto sceneMgr = parent->scene().manager();
    impulse_effects_.system = sceneMgr->createParticleSystem(parent->name() + "EngineJet", "SpaceEffects/JetEngine1");
    thruster_effects_.system = sceneMgr->createParticleSystem(parent->name() + "ThrusterSmoke", "SpaceEffects/Smoke");
    parent->node().attachObject(impulse_effects_.system);
    parent->node().attachObject(thruster_effects_.system);

    parent_body_ = parent->component<Body>();

    // add subsystems to corresponding particle systems.
    //TODO: will need to refactor this since we need to do this here for any existing subsystems,
    //      and at the AddSystem methods if owner() already exists.
    for (auto& engine : impulse_engines_) {
        AddEmitter(BtOgre::Convert::toOgre(engine->position()), engine->exhaust_direction(), engine->radius(), impulse_effects_);
    }
    for (auto& thruster : thrusters_) {
        AddEmitter(BtOgre::Convert::toOgre(thruster->position()), thruster->thrust_direction(), thruster->radius(), thruster_effects_);
    }
}
void Motion::AddEmitter(const Ogre::Vector3& pos, const Ogre::Vector3& dir, double radius, ParticleEffect& effect) {
    if (!system) return;
    Ogre::ParticleEmitter* emitter;
    if (effect.emitters.empty()) {
        emitter = effect.system->getEmitter(0);
        effect.emission_rate = emitter->getEmissionRate();
        effect.time_to_live = emitter->getTimeToLive();
        effect.velocity_min = emitter->getMinParticleVelocity();
        effect.velocity_max = emitter->getMaxParticleVelocity();
    }
    else {
        emitter = effect.system->addEmitter("Ring");
        effect.emitters[0]->copyParametersTo(emitter);
    }
    emitter->setPosition(pos);
    emitter->setDirection(dir);
    emitter->setParameter("width", std::to_string(radius * 2));
    emitter->setParameter("height", std::to_string(radius * 2));
    emitter->setParameter("depth", std::to_string(radius * 2));
    emitter->setParameter("inner_width", "0.05");
    emitter->setParameter("inner_height", "0.05");
    emitter->setEnabled(false); // best to start with all emitters offline, then update will turn on the required ones.
    effect.emitters.push_back(emitter);
}

void Motion::EnableEmitter(const ParticleEffect& effect, int emitter_index, double intensity) {
    auto emitter = effect.emitters[emitter_index];
    emitter->setEnabled(true);
    emitter->setParameter("emission_rate", std::to_string(effect.emission_rate * intensity));
    emitter->setParameter("time_to_live", std::to_string(effect.time_to_live * intensity));
    emitter->setParameter("velocity_min", std::to_string(effect.velocity_min * intensity));
    emitter->setParameter("velocity_max", std::to_string(effect.velocity_max * intensity));
}

} // namespace components
} // namespace shipsbattle
