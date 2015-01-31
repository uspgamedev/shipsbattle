
if (UNIX OR MINGW)
  add_definitions (-std=gnu++11)
endif ()

set(SHIPSBATTLE_SRC
shipsbattle/main.cc

shipsbattle/objects/objecttypes.h
shipsbattle/objects/ship.h
shipsbattle/objects/ship.cc
shipsbattle/objects/projectile.h
shipsbattle/objects/projectile.cc
shipsbattle/objects/projectilemodel.h
shipsbattle/objects/projectilemodel.cc
shipsbattle/objects/targets.h
shipsbattle/objects/targets.cc

shipsbattle/components/updateablecomponent.h
shipsbattle/components/updateablecomponent.cc
shipsbattle/components/spacedust.h
shipsbattle/components/spacedust.cc
shipsbattle/components/playercontroller.h
shipsbattle/components/playercontroller.cc
shipsbattle/components/projectilecontroller.h
shipsbattle/components/projectilecontroller.cc
shipsbattle/components/timedlife.h
shipsbattle/components/timedlife.cc
shipsbattle/components/hull.h
shipsbattle/components/hull.cc
shipsbattle/components/powersystem.h
shipsbattle/components/powersystem.cc
shipsbattle/components/tactical.h
shipsbattle/components/tactical.cc
shipsbattle/components/navigation.h
shipsbattle/components/navigation.cc
shipsbattle/components/motion.h
shipsbattle/components/motion.cc

shipsbattle/components/subsystems/typedefs.h
shipsbattle/components/subsystems/damageablesystem.h
shipsbattle/components/subsystems/damageablesystem.cc
shipsbattle/components/subsystems/subhull.h
shipsbattle/components/subsystems/powergenerator.h
shipsbattle/components/subsystems/battery.h
shipsbattle/components/subsystems/poweredsystem.h
shipsbattle/components/subsystems/poweredsystem.cc
shipsbattle/components/subsystems/weapon.h
shipsbattle/components/subsystems/weapon.cc
shipsbattle/components/subsystems/projectileweapon.h
shipsbattle/components/subsystems/projectileweapon.cc
shipsbattle/components/subsystems/sensorarray.h
shipsbattle/components/subsystems/sensorarray.cc
shipsbattle/components/subsystems/impulseengine.h
shipsbattle/components/subsystems/impulseengine.cc
shipsbattle/components/subsystems/thruster.h
shipsbattle/components/subsystems/thruster.cc

shipsbattle/utils/matrix.h
)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})
include_directories (${CMAKE_CURRENT_BINARY_DIR})

add_ugdk_executable(shipsbattle ${SHIPSBATTLE_SRC})
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -lXt -ldl")

