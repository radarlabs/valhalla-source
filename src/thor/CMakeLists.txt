file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/thor/*.h)

set(sources
  astar_bss.cc
  alternates.cc
  bidirectional_astar.cc
  costmatrix.cc
  dijkstras.cc
  matrix_action.cc
  multimodal.cc
  route_action.cc
  timedistancebssmatrix.cc
  timedistancematrix.cc
  triplegbuilder.cc
  unidirectional_astar.cc
  vehicle_routing.cc
  worker.cc)

set(sources_with_warnings
  centroid.cc
  expansion_action.cc
  isochrone_action.cc
  isochrone.cc
  map_matcher.cc
  optimized_route_action.cc
  optimizer.cc
  route_matcher.cc
  status_action.cc
  trace_attributes_action.cc
  trace_route_action.cc
  triplegbuilder_utils.h)

# Add OR-Tools include directory explicitly
set(system_includes
  ${date_include_dir}
  "/Users/boon.boonsiri/local/or-tools_arm64_macOS-15.3.1_cpp_v9.12.4544/include"
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>)

valhalla_module(NAME thor
  SOURCES ${sources}
  SOURCES_WITH_WARNINGS ${sources_with_warnings}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
    PRIVATE
      ${rapidjson_include_dir}
      ${unordered_dense_include_dir}
  LINK_LIBRARIES
    PUBLIC
      ortools::ortools
  DEPENDS
    valhalla::proto
    valhalla::sif
    valhalla::meili
    ${valhalla_protobuf_targets}
    Boost::boost
    ${libprime_server_targets}
    )
