
#include <ugdk/system/engine.h>
#include <ugdk/action/3D/camera.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/component/body.h>

#include <shipsbattle/objects/ship.h>
#include <shipsbattle/components/spacedust.h>
#include <shipsbattle/components/playercontroller.h>

#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/subhull.h>

#include <btBulletDynamicsCommon.h>
#include <OgreSceneManager.h>
#include <OgreOverlaySystem.h>

#include <memory>
#include <iostream>
#include <sstream>

#define AREA_RANGE 200.0

using std::weak_ptr;
using std::shared_ptr;
using std::unique_ptr;
using std::make_shared;
using std::stringstream;
using std::cout;
using std::endl;
using shipsbattle::objects::Ship;
using shipsbattle::components::SpaceDust;
using shipsbattle::components::PlayerController;

ugdk::action::mode3d::Scene3D *ourscene;

void CreateHUD(Ship& pla, Ship& enemy) {
    Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
    std::string over_name = "SBHUD";
    Ogre::Overlay* hud = overlay_mgr->create(over_name);
    hud->setZOrder(600);

    Ogre::TextAreaOverlayElement* plaHullStats = static_cast<Ogre::TextAreaOverlayElement*>(overlay_mgr->createOverlayElement("TextArea", over_name + "/PlaHullStats"));
    plaHullStats->setMetricsMode(Ogre::GMM_PIXELS);
    plaHullStats->setPosition(10, 0);
    plaHullStats->setDimensions(350, 30);
    plaHullStats->setFontName("testeFont");
    plaHullStats->setCharHeight(16);
    plaHullStats->setColour(Ogre::ColourValue::Black);
    plaHullStats->setCaption("PLAYER HULL STATS");
    Ogre::TextAreaOverlayElement* plaArmorStats = static_cast<Ogre::TextAreaOverlayElement*>(overlay_mgr->createOverlayElement("TextArea", over_name + "/PlaArmorStats"));
    plaArmorStats->setMetricsMode(Ogre::GMM_PIXELS);
    plaArmorStats->setPosition(10, 30);
    plaArmorStats->setDimensions(350, 30);
    plaArmorStats->setFontName("testeFont");
    plaArmorStats->setCharHeight(16);
    plaArmorStats->setColour(Ogre::ColourValue::Blue);
    plaArmorStats->setCaption("PLAYER ARMOR STATS");

    Ogre::TextAreaOverlayElement* enemyHullStats = static_cast<Ogre::TextAreaOverlayElement*>(overlay_mgr->createOverlayElement("TextArea", over_name + "/EnemyHullStats"));
    enemyHullStats->setMetricsMode(Ogre::GMM_PIXELS);
    enemyHullStats->setPosition(10, 60);
    enemyHullStats->setDimensions(350, 30);
    enemyHullStats->setFontName("testeFont");
    enemyHullStats->setCharHeight(16);
    enemyHullStats->setColour(Ogre::ColourValue::Black);
    enemyHullStats->setCaption("ENEMY HULL STATS");
    Ogre::TextAreaOverlayElement* enemyArmorStats = static_cast<Ogre::TextAreaOverlayElement*>(overlay_mgr->createOverlayElement("TextArea", over_name + "/EnemyArmorStats"));
    enemyArmorStats->setMetricsMode(Ogre::GMM_PIXELS);
    enemyArmorStats->setPosition(10, 90);
    enemyArmorStats->setDimensions(350, 30);
    enemyArmorStats->setFontName("testeFont");
    enemyArmorStats->setCharHeight(16);
    enemyArmorStats->setColour(Ogre::ColourValue::Blue);
    enemyArmorStats->setCaption("ENEMY ARMOR STATS");


    Ogre::OverlayContainer* panel = static_cast<Ogre::OverlayContainer*>(overlay_mgr->createOverlayElement("Panel", over_name + "/Panel"));
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setPosition(450, 0.0);
    panel->setDimensions(350, 120);
    panel->setMaterialName("BaseWhite");
    panel->addChild(plaHullStats);
    panel->addChild(plaArmorStats);
    panel->addChild(enemyHullStats);
    panel->addChild(enemyArmorStats);
    hud->add2D(panel);
    hud->show();

    ourscene->AddTask(ugdk::system::Task(
    [&pla, &enemy, plaHullStats, plaArmorStats, enemyHullStats, enemyArmorStats](double dt) {

        auto func = [](Ship& ship, Ogre::TextAreaOverlayElement* hullStats, Ogre::TextAreaOverlayElement* armorStats) {
            auto hull = ship.hull()->GetSubHull("MainHull");
            double hullPerc = 100.0 * hull->hitpoints() / hull->max_hitpoints();
            double armorPerc = 100.0 * hull->armor_rating() / hull->max_armor_rating();
            stringstream ssh;
            ssh.precision(2);
            ssh.setf(std::ios::fixed, std::ios::floatfield);
            ssh << ship->name() << " HULL: " << hullPerc << "% (" << hull->hitpoints() << "/" << hull->max_hitpoints() << ")";
            hullStats->setCaption(ssh.str());
            stringstream ssa;
            ssa.precision(2);
            ssa.setf(std::ios::fixed, std::ios::floatfield);
            ssa << ship->name() << " ARMOR: " << armorPerc << "% (" << hull->armor_rating() << "/" << hull->max_armor_rating() << ")";
            armorStats->setCaption(ssa.str());
        };

        func(pla, plaHullStats, plaArmorStats);
        func(enemy, enemyHullStats, enemyArmorStats);
    }));
}

Ship createShip(const std::string& name) {   
    return Ship(*ourscene, name, "AerOmar.mesh");
}

void AddSubHull(Ship& ship, const std::string& name, double radius, double x, double y, double z) {
    shipsbattle::components::Hull* hull = ship.hull();
    shipsbattle::components::subsystems::SubHull* sh = new shipsbattle::components::subsystems::SubHull(name);
    sh->SetVolume(radius, btVector3(x, y, z));
    hull->AddSubHull(std::shared_ptr<shipsbattle::components::subsystems::SubHull>(sh));
}

int main(int argc, char* argv[]) {
    ugdk::system::Configuration config;
    config.base_path = "assets/";
    config.windows_list.front().title = "Ships Battle";
    config.ogre_plugins.push_back("Plugin_ParticleFX");
    ugdk::system::Initialize(config);
    
    // create 3D scene
    ourscene = new ugdk::action::mode3d::Scene3D(btVector3(0.0, 0.0, 0.0));
    ourscene->manager()->setAmbientLight(Ogre::ColourValue(.7, .7, .7));
    // set some basic stuff our "space" scenes will need.
    ourscene->ShowFrameStats();
    ourscene->manager()->setSkyBox(true, "Backgrounds/Nebula1");
    
    {
        // create Player ship
        Ship player = createShip("Player");
        player->AddComponent(make_shared<SpaceDust>());
        player->AddComponent(make_shared<PlayerController>());
        ourscene->camera()->AttachTo(*player);
        ourscene->camera()->SetParameters(Ogre::Vector3::ZERO, 100);
        ourscene->camera()->SetDistance(10);

        //AerOmar bounding box size: (3.07287, 1.06726, 6.79639) - bounding sphere radius: 3.88078
        AddSubHull(player, "BallOrigin", 0.5, 0, 0, 0);
        //AddSubHull(player, "BallUno", 1.0, 0, 0, 1.0);
        //AddSubHull(player, "BallBottom", 0.1, 0, 0, 0.25);
        AddSubHull(player, "BallFront", 0.2, 0, 0, 3.7);

        ourscene->event_handler().AddListener<ugdk::input::KeyPressedEvent>(
        [&player](const ugdk::input::KeyPressedEvent& ev) -> void {
            if (ev.scancode == ugdk::input::Scancode::I) {
                player.hull()->TakeDamage(-100, 0.2, 5.0, btVector3(0, 0, 3.9));
            }
            else if (ev.scancode == ugdk::input::Scancode::O) {
                player.hull()->TakeDamage(100, 0.2, 1.0, btVector3(0, 0, 3.9));
            }
            else if (ev.scancode == ugdk::input::Scancode::P) {
                player.hull()->TakeDamage(100, 0.2, 0.0, btVector3(0, 0, 3.9));
            }
        });

        // create Enemy ship
        Ship enemy = createShip("Enemy");
        enemy.body()->Translate(0, 0, 80);

        CreateHUD(player, enemy);

        // push scene
        ugdk::system::PushScene(unique_ptr<ugdk::action::Scene>(ourscene));
    }

    ugdk::system::Run();
    ugdk::system::Release();
    return 0;
}
