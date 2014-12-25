
if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions (-std=gnu++11)
endif ()

set(SHIPSBATTLE_SRC
shipsbattle/main.cc
shipsbattle/components/spacedust.h
shipsbattle/components/spacedust.cc
shipsbattle/components/playercontroller.h
shipsbattle/components/playercontroller.cc
shipsbattle/components/subsystems/shipsubsystem.h
shipsbattle/components/subsystems/shipsubsystem.cc
shipsbattle/objects/objecttypes.h
shipsbattle/objects/ship.h
shipsbattle/objects/ship.cc
)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})
include_directories (${CMAKE_CURRENT_BINARY_DIR})

add_ugdk_executable(shipsbattle ${SHIPSBATTLE_SRC})
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -lXt -ldl")

