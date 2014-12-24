
#include <ugdk/system/engine.h>
#include <ugdk/desktop/module.h>
#include <ugdk/input/events.h>
#include <ugdk/input/module.h>
#include <ugdk/desktop/3D/manager.h>
#include <ugdk/action/3D/camera.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/physics.h>
#include <ugdk/action/3D/component/physicsbody.h>
#include <ugdk/action/3D/component/view.h>
#include <BtOgreGP.h>
#include <memory>
#include <iostream>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreMeshManager.h>

#include <shipsbattle/components/spacedust.h>

#define AREA_RANGE 200.0

using std::weak_ptr;
using std::shared_ptr;
using std::unique_ptr;
using std::make_shared;
using std::cout;
using std::endl;
using ugdk::action::mode3d::Element;
using ugdk::action::mode3d::component::PhysicsBody;
using ugdk::action::mode3d::component::Body;
using ugdk::action::mode3d::component::View;
using ugdk::action::mode3d::component::CollisionAction;
using ugdk::action::mode3d::component::ElementPtr;
using ugdk::action::mode3d::component::ManifoldPointVector;
using shipsbattle::components::SpaceDust;

ugdk::action::mode3d::Scene3D *ourscene;

#define BIT(x) (1<<(x))
enum CollisionGroup {
    HEADS = BIT(6),
    WALLS = BIT(7),
    WAT2 = BIT(8),
    WAT3 = BIT(9),
    WAT4 = BIT(10)
};

shared_ptr<Element> createOgreHead(const std::string& name, bool useBox=false) {
    Ogre::SceneManager *mSceneMgr = ourscene->manager();
    // Element
    auto head = ourscene->AddElement();
    // View
    auto headEnt = mSceneMgr->createEntity(name, "AerOmar.mesh");
    auto headSize = headEnt->getMesh()->getBounds().getSize();
    cout << name << " size: (" << headSize.x << ", " << headSize.y << ", " << headSize.z << ") - radius: " << headEnt->getBoundingRadius() << endl;
    head->AddComponent(make_shared<View>());
    head->component<View>()->AddEntity(headEnt);
    // Body
    PhysicsBody::PhysicsData headData;
    auto meshShapeConv = BtOgre::StaticMeshToShapeConverter(headEnt);
    if (useBox)
        headData.shape = meshShapeConv.createConvex();
    else
        headData.shape = meshShapeConv.createConvex();
    headData.mass = 80;
    headData.collision_group = CollisionGroup::HEADS;
    headData.collides_with = CollisionGroup::WALLS | CollisionGroup::HEADS;
    head->AddComponent(make_shared<PhysicsBody>(*ourscene->physics(), headData));
    head->component<Body>()->set_damping(.1, .1);
    head->component<Body>()->Scale(20.0,20.0,20.0);
    return head;
}

int main(int argc, char* argv[]) {
    ugdk::system::Configuration config;
    config.base_path = "assets/";
    config.windows_list.front().title = "Ships Battle";
    config.ogre_plugins.push_back("Plugin_ParticleFX");
    ugdk::system::Initialize(config);
    
    ourscene = new ugdk::action::mode3d::Scene3D(btVector3(0.0, 0.0, 0.0));
    
    ourscene->physics()->set_debug_draw_enabled(true);
    ourscene->ShowFrameStats();

    {
        weak_ptr<Element> head1 = createOgreHead("Head");
        weak_ptr<Element> head2 = createOgreHead("Head2", true);
        auto body2 = head2.lock()->component<Body>();
        body2->Translate(0, 0, 80);
        body2->set_angular_factor(0.0, 0.0, 0.0);

        head2.lock()->AddComponent(make_shared<SpaceDust>());

        body2->AddCollisionAction(CollisionGroup::HEADS, 
        [](const ElementPtr& self, const ElementPtr& target, const ManifoldPointVector& pts) {
            cout << "CARAS COLIDINDO MANO (" << pts.size() << ")" << endl;
        });

        ourscene->camera()->AttachTo(*head2.lock());
        ourscene->camera()->SetParameters(Ogre::Vector3::ZERO, 5000);
        ourscene->camera()->SetDistance(100);

        ourscene->manager()->setSkyBox(true, "Backgrounds/Nebula1");
        
        ourscene->AddTask(ugdk::system::Task(
        [body2](double dt) {
            auto& keyboard = ugdk::input::manager()->keyboard();
            Ogre::Vector3 move = Ogre::Vector3::ZERO;
            if (keyboard.IsDown(ugdk::input::Scancode::D))
                move.x += 1.0;
            else if (keyboard.IsDown(ugdk::input::Scancode::A))
                move.x += -1.0;
            if (keyboard.IsDown(ugdk::input::Scancode::W))
                move.z += -1.0;
            else if (keyboard.IsDown(ugdk::input::Scancode::S))
                move.z += 1.0;

            move.normalise();
            move = ourscene->camera()->actual_orientation() * move;
            move.normalise();
            
            body2->Move((move * 15));
        }));

        ourscene->event_handler().AddListener<ugdk::input::KeyPressedEvent>(
            [] (const ugdk::input::KeyPressedEvent& ev) -> void {
                if (ev.scancode == ugdk::input::Scancode::ESCAPE)
                    ourscene->Finish();
            });
        ourscene->event_handler().AddListener<ugdk::input::MouseMotionEvent>(
            [] (const ugdk::input::MouseMotionEvent& ev) -> void {
                ourscene->camera()->Rotate(-ev.motion.x, -ev.motion.y);
            });

        ourscene->manager()->setAmbientLight(Ogre::ColourValue(0.7, .7, .7));

        ugdk::system::PushScene(unique_ptr<ugdk::action::Scene>(ourscene));

    }

    ugdk::system::Run();
    ugdk::system::Release();
    return 0;
}
