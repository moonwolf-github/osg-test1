project(osg_test1)
cmake_minimum_required(VERSION 2.8)

# this would be better passed through cmake commandline parameters...
set(OSG_DIR "/home/moonwolf/Dokumenty/projects/osg/osg-3.2.2-rc2-bin")
find_package(OpenSceneGraph 3.2.2 REQUIRED osgDB osgUtil osgViewer osgGA osgTerrain)
# libOpenThreads & libosg automatically searched
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OPENSCENEGRAPH_LIBRARIES})
