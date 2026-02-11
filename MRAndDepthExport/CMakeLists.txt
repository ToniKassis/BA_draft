set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(_build_output_dir ${CMAKE_BINARY_DIR}/bin)
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${_build_output_dir})
endforeach(OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES)

set(_src_dir ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(_src_common_dir ${CMAKE_CURRENT_SOURCE_DIR}/../Common)

set(_sources_app
    ${_src_dir}/main.cpp
    ${_src_dir}/AppLogic.hpp
    ${_src_dir}/AppLogic.cpp
    ${_src_dir}/AppState.hpp
    ${_src_dir}/AppView.hpp
    ${_src_dir}/AppView.cpp
    ${_src_dir}/DataStreamer.hpp
    ${_src_dir}/DataStreamer.cpp
    ${_src_dir}/IPCConnectorHost.hpp
    ${_src_dir}/IPCConnectorHost.cpp
    ${_src_dir}/PointCloud.hpp
    ${_src_dir}/PointCloud.cpp
    ${_src_dir}/Depth16SequenceWriter.hpp
    ${_src_dir}/MRScene.hpp
    ${_src_dir}/MRScene.cpp
    ${_src_dir}/Visualization.hpp
)

set(_sources_common
    ${_src_common_dir}/CameraManager.hpp
    ${_src_common_dir}/CameraManager.cpp
    ${_src_common_dir}/D3D11Renderer.hpp
    ${_src_common_dir}/D3D11Renderer.cpp
    ${_src_common_dir}/D3D11Shaders.hpp
    ${_src_common_dir}/D3D11Shaders.cpp
    ${_src_common_dir}/D3D11MultiLayerView.hpp
    ${_src_common_dir}/D3D11MultiLayerView.cpp
    ${_src_common_dir}/ExampleShaders.hpp
    ${_src_common_dir}/GfxContext.hpp
    ${_src_common_dir}/GfxContext.cpp
    ${_src_common_dir}/Globals.hpp
    ${_src_common_dir}/Globals.cpp
    ${_src_common_dir}/MultiLayerView.hpp
    ${_src_common_dir}/MultiLayerView.cpp
    ${_src_common_dir}/HeadlessView.hpp
    ${_src_common_dir}/HeadlessView.cpp
    ${_src_common_dir}/Renderer.hpp
    ${_src_common_dir}/Renderer.cpp
    ${_src_common_dir}/Scene.hpp
    ${_src_common_dir}/Scene.cpp
    ${_src_common_dir}/SyncView.hpp
    ${_src_common_dir}/SyncView.cpp
    ${_src_common_dir}/UI.hpp
    ${_src_common_dir}/UI.cpp
    ${_src_common_dir}/Undistorter.hpp
    ${_src_common_dir}/Undistorter.cpp
)

source_group("Common" FILES ${_sources_common})

# Set options used by "FindBoost.cmake".
set (BOOST_INCLUDEDIR "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/boost/include")
set (Boost_NO_SYSTEM_PATHS ON)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

# COMPONENTS system filesystem
find_package (Boost 1.79.0 REQUIRED)
#find_package (PCL CONFIG REQUIRED PATHS C:/vcpkg/packages/pcl_x64-windows/share/pcl)
if (Boost_FOUND)
	include_directories (${Boost_INCLUDE_DIRS})

	set(_target MRAndDepthExport)
	add_executable(${_target} ${_sources_app} ${_sources_common})

	target_include_directories(${_target} PRIVATE ${_src_common_dir})
	set_property(TARGET ${_target} PROPERTY FOLDER "Projects")
	set_target_properties(${_target} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

	target_compile_definitions(${_target} PUBLIC -D_UNICODE -DUNICODE -DNOMINMAX)

	target_link_libraries(${_target}
  	  PRIVATE ImGui::ImGui
 	  PRIVATE ImGui::ImGuiBackends
  	  PRIVATE GLM
  	  PRIVATE CxxOpts
  	  PRIVATE VarjoLib
 	  PRIVATE d3d11
 	  PRIVATE dxgi
	  PRIVATE d3dcompiler
 	  PRIVATE windowscodecs
	  PRIVATE OpenCV
    #  PRIVATE ${PCL_LIBRARIES}
 	  PRIVATE ${Boost_LIBRARIES}
	)

endif ()
