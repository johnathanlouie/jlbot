CMAKE_MINIMUM_REQUIRED (VERSION 3.1 FATAL_ERROR)
PROJECT (jlbot)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")
INCLUDE (UsePlayerC++)

SET (HAVE_GETOPT 1)
IF (NOT HAVE_GETOPT)
    SET (replaceLib "-lplayerreplace")
ENDIF (NOT HAVE_GETOPT)

IF (PLAYER_OS_SOLARIS)
    SET (rtLibFlag -lrt)
ENDIF (PLAYER_OS_SOLARIS)

PLAYER_ADD_PLAYERCPP_CLIENT (
  jlbot SOURCES
  src/main.cc
  src/actors.cc
  src/planners.cc
  src/sensors.cc
  src/misc.cc
  src/worldmodel.cc
  LINKFLAGS ${replaceLib}
)
#PLAYER_ADD_PLAYERCPP_CLIENT (camera SOURCES camera.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (example0 SOURCES example0.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (example4 SOURCES example4.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (grip SOURCES grip.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (clientgraphics SOURCES clientgraphics.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (clientgraphics3d SOURCES clientgraphics3d.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (laserobstacleavoid SOURCES laserobstacleavoid.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (ptz SOURCES ptz.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (randomwalk SOURCES randomwalk.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (sonarobstacleavoid SOURCES sonarobstacleavoid.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (speech SOURCES speech.cc LINKFLAGS ${replaceLib})
#PLAYER_ADD_PLAYERCPP_CLIENT (wallfollow SOURCES wallfollow.cc LINKFLAGS ${replaceLib})

SET (HAVE_BOOST_THREAD TRUE)
SET (HAVE_BOOST_SIGNALS TRUE)

#IF (HAVE_BOOST_THREAD AND HAVE_BOOST_SIGNALS)
#    PLAYER_ADD_PLAYERCPP_CLIENT (example1 SOURCES example1.cc LINKFLAGS ${rtLibFlag} LINKFLAGS ${replaceLib})
#    PLAYER_ADD_PLAYERCPP_CLIENT (example3 SOURCES example3.cc LINKFLAGS ${replaceLib})
#    PLAYER_ADD_PLAYERCPP_CLIENT (goto SOURCES goto.cc LINKFLAGS ${rtLibFlag} LINKFLAGS ${replaceLib})
#    PLAYER_ADD_PLAYERCPP_CLIENT (speech_cpp_client SOURCES speech_cpp_client.cc LINKFLAGS ${rtLibFlag} LINKFLAGS ${replaceLib})
#ENDIF (HAVE_BOOST_THREAD AND HAVE_BOOST_SIGNALS)

#IF (HAVE_BOOST_SIGNALS)
#    PLAYER_ADD_PLAYERCPP_CLIENT (example2 SOURCES example2.cc LINKFLAGS ${replaceLib})
#ENDIF (HAVE_BOOST_SIGNALS)
