
#include <ugdk/system/engine.h>
#include <ugdk/desktop/module.h>
#include <ugdk/desktop/3D/manager.h>
#include <ugdk/action/3D/camera.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/physics.h>
#include <ugdk/action/3D/component/physicsbody.h>
#include <ugdk/action/3D/component/view.h>

#include <shipsbattle/components/spacedust.h>
#include <shipsbattle/components/playercontroller.h>

#include <BtOgreGP.h>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreMeshManager.h>

#include <memory>
#include <iostream>

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
using shipsbattle::components::PlayerController;

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
    //head->component<Body>()->Scale(10.0,10.0,10.0);
    return head;
}

int main(int argc, char* argv[]) {
    ugdk::system::Configuration config;
    config.base_path = "assets/";
    config.windows_list.front().title = "Ships Battle";
    config.ogre_plugins.push_back("Plugin_ParticleFX");
    ugdk::system::Initialize(config);
    
    ourscene = new ugdk::action::mode3d::Scene3D(btVector3(0.0, 0.0, 0.0));
    ourscene->ShowFrameStats();

    {
        weak_ptr<Element> head1 = createOgreHead("Head");
        weak_ptr<Element> head2 = createOgreHead("Head2", true);
        auto body2 = head2.lock()->component<Body>();
        body2->Translate(0, 0, 80);
        //body2->set_angular_factor(1.0, 1.0, 0.0);

        head2.lock()->AddComponent(make_shared<SpaceDust>());
        head2.lock()->AddComponent(make_shared<PlayerController>());

        body2->AddCollisionAction(CollisionGroup::HEADS, 
        [](const ElementPtr& self, const ElementPtr& target, const ManifoldPointVector& pts) {
            cout << "CARAS COLIDINDO MANO (" << pts.size() << ")" << endl;
        });

        ourscene->camera()->AttachTo(*head2.lock());
        ourscene->camera()->SetParameters(Ogre::Vector3::ZERO, 100);
        ourscene->camera()->SetDistance(10);

        ourscene->manager()->setSkyBox(true, "Backgrounds/Nebula1");
        
        
        ourscene->manager()->setAmbientLight(Ogre::ColourValue(.7, .7, .7));

        ugdk::system::PushScene(unique_ptr<ugdk::action::Scene>(ourscene));

    }

    ugdk::system::Run();
    ugdk::system::Release();
    return 0;
}
