#ifndef SHIPSBATTLE_OBJECTS_TARGETS_H_
#define SHIPSBATTLE_OBJECTS_TARGETS_H_

#include <vector>
#include <unordered_set>
#include <string>
#include <memory>

namespace ugdk {
namespace action {
namespace mode3d {
class Element;
}}}

namespace shipsbattle {
namespace components {
namespace subsystems {
class DamageableSystem;
}}
namespace objects {

/** Class that represents a actual target element for ship or weapon, pointing to
another element and a damageable system of it. */
class Target {
public:
    Target(const std::weak_ptr<ugdk::action::mode3d::Element>& obj, 
        components::subsystems::DamageableSystem* sys) : object_(obj), system_(sys) {}

    bool valid() const {
        return !object_.expired();
    }
    
    operator components::subsystems::DamageableSystem*() const;
    components::subsystems::DamageableSystem* operator->() const { return system_; }

protected:
    std::weak_ptr<ugdk::action::mode3d::Element> object_;
    components::subsystems::DamageableSystem* system_;
};

/** Class which represents a targeteable game element, allowing to select 
sub-targets (subsystems) in this object. */
class TargetData {
public:
    TargetData(const std::weak_ptr<ugdk::action::mode3d::Element>& obj) : object_(obj) {}

    bool valid() const {
        return !object_.expired();
    }

    std::vector<Target> GetPossibleTargets() const;
    void ToggleTarget(const std::string& sys_name, bool selected);
    std::vector<Target> GetTargets();
    bool IsTargeted(const std::string& sys_name) const;
    bool IsTargeted(components::subsystems::DamageableSystem* sys) const;

    operator Target() const;
    std::shared_ptr<ugdk::action::mode3d::Element> object() const {
        return object_.lock();
    }

protected:
    std::weak_ptr<ugdk::action::mode3d::Element> object_;
    std::unordered_set<std::string> subsystems_;
};

/** Set of Targets. Holds a list of TargetData, which represents a targeteable game element. */
using TargetSet = std::vector < std::shared_ptr<TargetData> >;

} // namespace objects
} // namespace shipsbattle
#endif //SHIPSBATTLE_OBJECTS_TYPES_H_
