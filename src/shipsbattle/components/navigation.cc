#include <shipsbattle/components/navigation.h>
#include <shipsbattle/components/subsystems/sensorarray.h>
#include <shipsbattle/objects/objecttypes.h>
#include <shipsbattle/objects/ship.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/physics.h>
#include <ugdk/debug/log.h>

#include <LinearMath/btVector3.h>

#include <unordered_set>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::SensorArray;
using shipsbattle::objects::Ship;
using shipsbattle::objects::TargetData;
using shipsbattle::objects::TargetSet;

namespace shipsbattle {
namespace components {

void Navigation::AddSensorArray(const shared_ptr<SensorArray>& sensor) {
    if (sensor_indexes_.count(sensor->name()) > 0) {
        Log(LogLevel::WARNING, "Navigation System", "SensorArray with name '" + sensor->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this SensorArray.");
        return;
    }
    sensors_.push_back(sensor);
    sensor_indexes_[sensor->name()] = sensors_.size() - 1;
    sensor->RegisteredTo(this);
}
const shared_ptr<SensorArray>& Navigation::GetSensorArray(size_t index) {
    return sensors_.at(index);
}
const shared_ptr<SensorArray>& Navigation::GetSensorArray(const std::string& name) {
    return GetSensorArray(sensor_indexes_[name]);
}

void Navigation::Update(double dt) {
    for (auto sensor : sensors_) {
        sensor->Update(dt);

        if (sensor->elapsed_ > sensor->refresh_rate_) {
            sensor->elapsed_ = 0.0;
            // run sensor sweep
            auto sensor_data = owner()->scene().physics()->ContactQuery(objects::ObjectTypes::SHIP, sensor->world_position(), sensor->maximum_range());
            
            // get old list of object keys
            std::unordered_set<std::string> obj_keys;
            obj_keys.reserve(objects_.size());
            for (auto kv : objects_) {
                obj_keys.insert(kv.first);
            }
            // update object map with new objects
            for (auto weak_obj : sensor_data) {
                auto obj = weak_obj.lock();
                if (obj->marked_for_removal()) continue;
                if (obj->name() == owner()->name()) continue;
                auto it_key = obj_keys.find(obj->name());
                if (it_key == obj_keys.end()) {
                    // obj is not in our old objects_ map.
                    objects_[obj->name()] = std::make_shared<TargetData>(obj);
                }
                else {
                    // obj is already on our old objects_ map, no need to do anything with it.
                    obj_keys.erase(it_key);
                }
            }
            // update object map removing old objects
            for (auto key : obj_keys) {
                objects_.erase(key);
                targets_.erase(key);
            }

            /*
            -navigation (isso aqui) que tem que manter info de alvos (lista e atuais), com interface pra mudar e listar alvos.
            -ai os controllers (no caso soh o PlayerController) simplesmente usa essa interface pra mudar alvos.
            -criar uma estrutura TargetData que guarda info de alvos (aqui no navigation) e eh usada para passar alvos 
             para os lugares que precisam, como as Weapons, projectile e Projectile controller
                -ele tem que ter como "alvo" um weak_ptr<Element> ou uma posição no espaço
                -se o weak_ptr fica expired, entao usa ultima posicao conhecida como alvo, ou só continua indo reto.
                -com isso deve deixar um alvo ser uma nave de fato, um ponto no espaco ou talvez soh atirar numa direção X (dir da arma?)
                -Como TargetData serão dados do Navigation de uma nave, ele pode ter algumas outras informacoes setadas pela nave, como
                 prioridade do alvo, e quais armas ja atiraram nesse alvo (em qual periodo de tempo?)
            -Os TargetData em si devem se expiráveis tb. Ou algo similar, para que se nave A mira na nave B e a B some dos sensores, ela
             nao pode continuar atirando na B (armas ja em voo podem continuar). Porem eh bom ter uma memoria de curto prazo de alvos anteriores,
             pra se a B sumir por pouco tempo e voltar, a mira da A volta para a B (se não foi alterada manualmente)
            -lembrar que os alvos não são naves, mas sim os DamageableSystems delas. Entao um unico alvo é um sistema. Mas podemos ter como alvo
             diversos sistemas, de diversas naves.
            */
        }
    }
}

TargetSet Navigation::GetSensorObjects() {
    TargetSet targets;
    targets.reserve(objects_.size());
    for (auto kv : objects_) {
        targets.push_back(kv.second);
    }
    return targets;
}

std::shared_ptr<objects::TargetData> Navigation::ToggleTarget(const std::string& target_name, bool selected) {
    auto tkey = targets_.find(target_name);
    bool in_targets = tkey != targets_.end();
    if (selected && !in_targets) {
        targets_.insert(target_name);
    }
    else if (!selected && in_targets) {
        targets_.erase(tkey);
    }
    auto tdata = objects_.find(target_name);
    if (tdata == objects_.end()) return std::shared_ptr<objects::TargetData>();
    return (*tdata).second;
}

TargetSet Navigation::GetTargets() {
    TargetSet targets;
    targets.reserve(targets_.size());
    for (auto key : targets_) {
        auto it = objects_.find(key);
        if (it != objects_.end())
            targets.push_back( it->second );
    }
    return targets;
}

bool Navigation::IsTargeted(const std::string& target_name) const {
    return targets_.count(target_name) >= 0;
}

} // namespace components
} // namespace shipsbattle
