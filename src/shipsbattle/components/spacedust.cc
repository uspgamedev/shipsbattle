
#include <shipsbattle/components/spacedust.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/camera.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreParticleSystem.h>
#include <OgreParticle.h>

namespace shipsbattle {
namespace components {

void SpaceDust::OnTaken() {
    auto parent = owner();
    auto sceneMgr = parent->node().getCreator();
    auto& scene = parent->scene();

    dust_system_ = sceneMgr->createParticleSystem("SpaceDust", "SpaceEffects/Dust");
    parent->node().attachObject(dust_system_);

    scene.AddTask(ugdk::system::Task(
        [this](double dt) {
        const float maxDist = 250.0;
        const float mirrorDist = maxDist*0.99;
        const float dimFactor = 0.8*0.005*0.005;
        const float maxDist2 = maxDist*maxDist;
        Ogre::Camera* cam = this->owner()->scene().camera()->camera();
        const Ogre::Vector3& camPos = cam->getRealPosition();

        const float twiceMaxDist = 2 * maxDist;

        Ogre::ParticleIterator pit = this->dust_system_->_getIterator();

        while (!pit.end())
        {
            Ogre::Particle* particle = pit.getNext();
            Ogre::Vector3& pos = particle->position;
            particle->timeToLive = 999999.0f;

            // position particles near camera
            // (keep moving them toward camera until within range)
            while (pos.x - camPos.x > maxDist)
                pos.x -= twiceMaxDist;
            while (pos.x - camPos.x < -maxDist)
                pos.x += twiceMaxDist;
            while (pos.y - camPos.y > maxDist)
                pos.y -= twiceMaxDist;
            while (pos.y - camPos.y < -maxDist)
                pos.y += twiceMaxDist;
            while (pos.z - camPos.z > maxDist)
                pos.z -= twiceMaxDist;
            while (pos.z - camPos.z < -maxDist)
                pos.z += twiceMaxDist;

            Ogre::Vector3 pDir = pos - camPos;
            float dist = pDir.squaredLength();
            float dim = dist*dimFactor;
            particle->setDimensions(dim, dim);
        }
    }));
}

} // namespace components
} // namespace shipsbattle
