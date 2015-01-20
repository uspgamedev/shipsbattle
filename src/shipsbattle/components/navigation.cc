#include <shipsbattle/components/navigation.h>
#include <shipsbattle/components/subsystems/sensorarray.h>
#include <shipsbattle/objects/objecttypes.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/physics.h>
#include <ugdk/debug/log.h>

#include <LinearMath/btVector3.h>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::SensorArray;

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
            auto ships = owner()->scene().physics()->ContactQuery(objects::ObjectTypes::SHIP, sensor->world_position(), sensor->maximum_range());
            Log(LogLevel::INFO, "Navigation System", "<<< RUNNING SENSOR SWEEP >>>");
            for (auto ship : ships) {
                Log(LogLevel::INFO, "Navigation System", "Detected ship '" + ship.lock()->name() + "'");
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

} // namespace components
} // namespace shipsbattle
