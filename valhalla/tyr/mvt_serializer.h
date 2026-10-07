#pragma once

#include <string>
#include <memory>
#include <vector>

// Forward declaration for vtzero
namespace vtzero {
  class layer_builder;
}
#include <utility>

// Include the necessary Valhalla headers
#include "proto/api.pb.h"
#include "proto/options.pb.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "baldr/graphreader.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/nodeinfo.h"

namespace valhalla {
namespace tyr {

/**
 * MVT (Mapbox Vector Tiles) serializer for Valhalla routing data
 */
class MvtSerializer {
public:
  /**
   * Serialize routing data to MVT format
   * @param api The API response containing routing data
   * @param format The output format (should be MVT)
   * @param graph_reader The graph reader to access routing data (optional, will create if not provided)
   * @param config The configuration for zoom-based filtering (optional)
   * @return MVT data as a string
   */
  static std::string serialize(const valhalla::Api& api, const valhalla::Options_Format& format,
                              const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader = nullptr,
                              const boost::property_tree::ptree* config = nullptr);


private:
  /**
   * Calculate the lat/lng bounds for a tile at given z/x/y coordinates
   * @param z The zoom level
   * @param x The tile x coordinate
   * @param y The tile y coordinate
   * @return Bounding box as AABB2<PointLL>
   */
  static valhalla::midgard::AABB2<valhalla::midgard::PointLL> calculateTileBounds(uint32_t z, uint32_t x, uint32_t y);

  /**
   * Generate MVT protobuf data for a tile
   * @param z The zoom level
   * @param x The tile x coordinate
   * @param y The tile y coordinate
   * @param bbox The tile bounding box
   * @param graph_reader The graph reader to access routing data (optional)
   * @param config The configuration for zoom-based filtering (optional)
   * @return MVT protobuf data as binary string
   */
  static std::string generateMvtProtobuf(uint32_t z, uint32_t x, uint32_t y,
                                        const valhalla::midgard::AABB2<valhalla::midgard::PointLL>& bbox,
                                        const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader = nullptr,
                                        const boost::property_tree::ptree* config = nullptr,
                                        const valhalla::Api& api = valhalla::Api());

  static constexpr uint32_t MVT_TILE_SIZE = 4096;


};

} // namespace tyr
} // namespace valhalla
