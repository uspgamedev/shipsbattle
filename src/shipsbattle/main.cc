
#include <ugdk/system/engine.h>
#include <ugdk/action/3D/camera.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/component/body.h>

#include <shipsbattle/objects/ship.h>
#include <shipsbattle/components/spacedust.h>
#include <shipsbattle/components/playercontroller.h>
#include <shipsbattle/components/subsystems/typedefs.h>
#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/subhull.h>
#include <shipsbattle/components/navigation.h>
#include <shipsbattle/components/timedlife.h>

#include <btBulletDynamicsCommon.h>
#include <OgreSceneManager.h>
#include <OgreOverlaySystem.h>

#include <OgreMeshManager.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSubMesh.h>

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
using shipsbattle::components::Navigation;
using namespace shipsbattle::components::subsystems::DecaymentFunctions;

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
    panel->setPosition(674, 0.0);
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
            if (ship.valid()) {
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
            }
            else {
                hullStats->setCaption("HULL: DESTROYED");
                armorStats->setCaption("ARMOR: DESTROYED");
            }
        };
        
        func(pla, plaHullStats, plaArmorStats);
        func(enemy, enemyHullStats, enemyArmorStats);
    }));
}

void createSphere(const std::string& name, const float r, const int nRings=16, const int nSegments=16) {
    Ogre::MeshPtr pSphere = Ogre::MeshManager::getSingleton().createManual(Ogre::String(name), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::SubMesh *pSphereVertex = pSphere->createSubMesh();

    pSphere->sharedVertexData = new Ogre::VertexData();
    Ogre::VertexData* vertexData = pSphere->sharedVertexData;

    // define the vertex format
    Ogre::VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
    size_t currOffset = 0;
    // positions
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    // normals
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    // two dimensional texture coordinates
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

    // allocate the vertex buffer
    vertexData->vertexCount = (nRings + 1) * (nSegments + 1);
    Ogre::HardwareVertexBufferSharedPtr vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
    Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
    binding->setBinding(0, vBuf);
    float* pVertex = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // allocate index buffer
    pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
    pSphereVertex->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
    Ogre::HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
    unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    float fDeltaRingAngle = (Ogre::Math::PI / nRings);
    float fDeltaSegAngle = (2 * Ogre::Math::PI / nSegments);
    unsigned short wVerticeIndex = 0;

    // Generate the group of rings for the sphere
    for (int ring = 0; ring <= nRings; ring++) {
        float r0 = r * sinf(ring * fDeltaRingAngle);
        float y0 = r * cosf(ring * fDeltaRingAngle);

        // Generate the group of segments for the current ring
        for (int seg = 0; seg <= nSegments; seg++) {
            float x0 = r0 * sinf(seg * fDeltaSegAngle);
            float z0 = r0 * cosf(seg * fDeltaSegAngle);

            // Add one vertex to the strip which makes up the sphere
            *pVertex++ = x0;
            *pVertex++ = y0;
            *pVertex++ = z0;

            Ogre::Vector3 vNormal = Ogre::Vector3(x0, y0, z0).normalisedCopy();
            *pVertex++ = vNormal.x;
            *pVertex++ = vNormal.y;
            *pVertex++ = vNormal.z;

            *pVertex++ = (float)seg / (float)nSegments;
            *pVertex++ = (float)ring / (float)nRings;

            if (ring != nRings) {
                // each vertex (except the last) has six indices pointing to it
                *pIndices++ = wVerticeIndex + nSegments + 1;
                *pIndices++ = wVerticeIndex;
                *pIndices++ = wVerticeIndex + nSegments;
                *pIndices++ = wVerticeIndex + nSegments + 1;
                *pIndices++ = wVerticeIndex + 1;
                *pIndices++ = wVerticeIndex;
                wVerticeIndex++;
            }
        }; // end for seg
    } // end for ring

    // Unlock
    vBuf->unlock();
    iBuf->unlock();
    // Generate face list
    pSphereVertex->useSharedVertices = true;

    // the original code was missing this line:
    pSphere->_setBounds(Ogre::AxisAlignedBox(Ogre::Vector3(-r, -r, -r), Ogre::Vector3(r, r, r)), false);
    pSphere->_setBoundingSphereRadius(r);
    // this line makes clear the mesh is loaded (avoids memory leaks)
    pSphere->load();

    //material
    Ogre::MaterialPtr mMat = Ogre::MaterialManager::getSingleton().create("AmmoSphere", "General", true);
    mMat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(1.0f, 0.1f, 0.0f));
    mMat->getTechnique(0)->getPass(0)->setEmissive(Ogre::ColourValue(1.0f, 0.1f, 0.0f));
    pSphereVertex->setMaterialName("AmmoSphere");
}

Ship createShip(const std::string& name) {   
    return Ship(*ourscene, name, "AerOmarOld.mesh");
}

void AddSubHull(Ship& ship, const std::string& name, double radius, double x, double y, double z) {
    shipsbattle::components::Hull* hull = ship.hull();
    shipsbattle::components::subsystems::SubHull* sh = new shipsbattle::components::subsystems::SubHull(name);
    sh->SetVolume(radius, btVector3(static_cast<btScalar>(x), static_cast<btScalar>(y), static_cast<btScalar>(z)));
    hull->AddSubHull(std::shared_ptr<shipsbattle::components::subsystems::SubHull>(sh));
}

int main(int argc, char* argv[]) {
    ugdk::system::Configuration config;
    config.base_path = "assets/";
    config.windows_list.front().title = "Ships Battle";
    config.windows_list.front().size = ugdk::math::Integer2D(1024, 768);
    config.ogre_plugins.push_back("Plugin_ParticleFX");
    ugdk::system::Initialize(config);
    
    // create 3D scene
    ourscene = new ugdk::action::mode3d::Scene3D(btVector3(0.0, 0.0, 0.0));
    ourscene->manager()->setAmbientLight(Ogre::ColourValue(.7f, .7f, .7f));
    // set some basic stuff our "space" scenes will need.
    ourscene->ShowFrameStats();
    ourscene->manager()->setSkyBox(true, "Backgrounds/Nebula1");
    
    // prepare our simple test mesh for projectiles
    createSphere("Ammo", 0.1f);

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

    // create Enemy ship
    Ship enemy = createShip("Enemy");
    enemy.body()->Translate(0, 0, 80);
    AddSubHull(enemy, "Butt", 0.2, 0.0, 0.0, -3.6);
    player->component<Navigation>()->ToggleTarget("Enemy", true);
    //enemy->AddComponent(std::make_shared<shipsbattle::components::TimedLife>(15.0));

    CreateHUD(player, enemy);
    ourscene->event_handler().AddListener<ugdk::input::KeyPressedEvent>(
        [&player,&enemy](const ugdk::input::KeyPressedEvent& ev) -> void {
        if (ev.scancode == ugdk::input::Scancode::I) {
            player.hull()->TakeDamage(-100, 0.2, 5.0, btVector3(0, 0, 3.9), CONSTANT);
        }
        else if (ev.scancode == ugdk::input::Scancode::O) {
            player.hull()->TakeDamage(100, 0.2, 1.0, btVector3(0, 0, 3.9), CONSTANT);
        }
        else if (ev.scancode == ugdk::input::Scancode::P) {
            player.hull()->TakeDamage(100, 0.2, 0.0, btVector3(0, 0, 3.9), CONSTANT);
        }
        else if (ev.scancode == ugdk::input::Scancode::SPACE) {
            enemy.hull()->TakeDamage(-500.0, 0.0, 1.0, btVector3(0, 0, 0), CONSTANT);
        }
    });

    // push scene
    ugdk::system::PushScene(unique_ptr<ugdk::action::Scene>(ourscene));

    ugdk::system::Run();
    ugdk::system::Release();
    return 0;
}
