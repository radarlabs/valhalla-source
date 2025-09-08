#include "tyr/mvt_serializer.h"
#include "proto/api.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/nodeinfo.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "midgard/pointll.h"
#include "midgard/aabb2.h"
#include "midgard/util.h"
#include "loki/node_search.h"

// protozero is required by vtzero
#include "third_party/protozero/include/protozero/pbf_reader.hpp"
// vtzero for MVT encoding
#include "third_party/vtzero/include/vtzero/builder.hpp"

#include <sstream>
#include <vector>
#include <memory>
#include <iomanip>
#include <cmath>
#include <map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace valhalla {
namespace tyr {


std::string MvtSerializer::serialize(const valhalla::Api& api, const valhalla::Options_Format& format,
                                    const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader) {
  LOG_INFO("MVT DEBUG: serialize called with format: " + std::to_string(format));

  // Currently only support MVT format
  if (format != valhalla::Options_Format_mvt) {
    LOG_ERROR("MVT DEBUG: Unsupported format: " + std::to_string(format));
    throw std::runtime_error("MVT serialization requires MVT format");
  }

  LOG_INFO("MVT DEBUG: Format check passed, generating tile");

  // Parse tile coordinates from the API options id field
  // The id field contains "z/x/y" coordinates from the tile endpoint
  std::string tile_id = api.options().id();
  LOG_INFO("MVT DEBUG: Tile ID from API: " + tile_id);

  if (tile_id.empty() || tile_id == "tile_endpoint_active") {
    LOG_INFO("MVT DEBUG: No specific tile coordinates, returning placeholder");
    return "MVT format supported - use tile endpoint for actual tile generation";
  }

  // Parse z/x/y coordinates from tile_id (format: "z/x/y")
  std::vector<std::string> parts;
  std::stringstream ss(tile_id);
  std::string part;

  while (std::getline(ss, part, '/')) {
    parts.push_back(part);
  }

  if (parts.size() != 3) {
    LOG_ERROR("MVT DEBUG: Invalid tile ID format: " + tile_id);
    return "Invalid tile coordinates";
  }

  try {
    uint32_t z = std::stoul(parts[0]);
    uint32_t x = std::stoul(parts[1]);
    uint32_t y = std::stoul(parts[2]);

    LOG_INFO("MVT DEBUG: Parsed coordinates: z=" + std::to_string(z) + ", x=" + std::to_string(x) + ", y=" + std::to_string(y));

    // Calculate tile bounds (will be used for actual tile generation later)
    auto bbox = calculateTileBounds(z, x, y);
    (void)bbox; // Suppress unused variable warning for now
    LOG_INFO("MVT DEBUG: Calculated tile bounds");

          // Generate actual MVT protobuf data
      LOG_INFO("MVT DEBUG: Generating MVT protobuf data");
      std::string mvt_data = generateMvtProtobuf(z, x, y, bbox, graph_reader);
      LOG_INFO("MVT DEBUG: Generated MVT protobuf, size: " + std::to_string(mvt_data.size()));
      return mvt_data;

  } catch (const std::exception& e) {
    LOG_ERROR("MVT DEBUG: Failed to parse tile coordinates: " + std::string(e.what()));
    return "Failed to parse tile coordinates";
  }
}

std::string MvtSerializer::generateTile(const valhalla::midgard::AABB2<valhalla::midgard::PointLL>& bbox,
                                       uint32_t zoom,
                                       const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader) {
  std::ostringstream oss;

  // Start MVT tile structure (simplified for now)
  oss << "{\"type\":\"FeatureCollection\",\"features\":[";

  // Get tiles in the bounding box
  auto tiles = graph_reader->GetTileSet();
  bool first_feature = true;

  for (const auto& tile_id : tiles) {
    auto tile = graph_reader->GetGraphTile(tile_id);
    if (!tile) continue;

    // Check if tile intersects with bounding box
    auto tile_bbox = tile->BoundingBox();
    if (!bbox.Intersects(tile_bbox)) continue;

    // Process edges in the tile
    for (uint32_t i = 0; i < tile->header()->directededgecount(); ++i) {
      auto edge = tile->directededge(i);
      if (!edge) continue;

      // Get edge info
      auto edge_info = tile->edgeinfo(edge);

      // Get edge shape and convert to MVT coordinates
      auto shape = edge_info.shape();
      if (shape.empty()) continue;

      std::vector<std::pair<int32_t, int32_t>> mvt_coords;
      for (const auto& point : shape) {
        auto coords = pointToMvtCoords(point, bbox, zoom);
        mvt_coords.push_back(coords);
      }

      if (!mvt_coords.empty()) {
        if (!first_feature) oss << ",";
        std::string feature = createEdgeFeature(mvt_coords, edge, &edge_info);
        oss << feature;
        first_feature = false;
      }
    }

    // Process nodes in the tile
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i) {
      auto node = tile->node(i);
      if (!node) continue;

      // Get node coordinates (need tile corner for latlng)
      auto tile_corner = tile->BoundingBox().minpt();
      auto point = node->latlng(tile_corner);

      // Check if node is within bounding box
      if (bbox.Contains(point)) {
        auto mvt_coords = pointToMvtCoords(point, bbox, zoom);

        if (!first_feature) oss << ",";
        std::string feature = createNodeFeature(mvt_coords, node);
        oss << feature;
        first_feature = false;
      }
    }
  }

  oss << "]}";
  return oss.str();
}

std::pair<int32_t, int32_t> MvtSerializer::pointToMvtCoords(
    const valhalla::midgard::PointLL& point,
    const valhalla::midgard::AABB2<valhalla::midgard::PointLL>& bbox,
    uint32_t zoom) {

  // Convert lat/lng to MVT tile coordinates (0-4096)
  // This should be relative to the tile bounds, not global tile coordinates

  double lat = point.lat();
  double lng = point.lng();

  // Get tile bounds
  double min_lat = bbox.miny();
  double max_lat = bbox.maxy();
  double min_lng = bbox.minx();
  double max_lng = bbox.maxx();

  // Convert to MVT coordinates (0-4096)
  // Normalize the point within the tile bounds
  double x_ratio = (lng - min_lng) / (max_lng - min_lng);
  double y_ratio = (lat - min_lat) / (max_lat - min_lat);

  // Note: MVT uses y=0 at the top, so we invert the y coordinate
  int32_t x = static_cast<int32_t>(x_ratio * MVT_TILE_SIZE);
  int32_t y = static_cast<int32_t>((1.0 - y_ratio) * MVT_TILE_SIZE);

  // Clamp to tile bounds
  x = std::max(0, std::min(x, static_cast<int32_t>(MVT_TILE_SIZE - 1)));
  y = std::max(0, std::min(y, static_cast<int32_t>(MVT_TILE_SIZE - 1)));

  return {x, y};
}

std::string MvtSerializer::createEdgeFeature(const std::vector<std::pair<int32_t, int32_t>>& coords,
                                            const valhalla::baldr::DirectedEdge* edge,
                                            const valhalla::baldr::EdgeInfo* edge_info) {
  std::ostringstream oss;

  oss << "{\"type\":\"Feature\",\"geometry\":{";
  oss << "\"type\":\"LineString\",\"coordinates\":[";

  for (size_t i = 0; i < coords.size(); ++i) {
    if (i > 0) oss << ",";
    oss << "[" << coords[i].first << "," << coords[i].second << "]";
  }

  oss << "]},\"properties\":{";
  oss << "\"edge_id\":" << edge->edgeinfo_offset() << ",";
  oss << "\"road_class\":" << static_cast<int>(edge->classification()) << ",";
  oss << "\"speed\":" << edge->speed() << ",";
  oss << "\"oneway\":" << (edge->forward() && !edge->reverseaccess()) << ",";
  auto names = edge_info->GetNames();
  std::string name = names.empty() ? "unnamed" : names[0];
  oss << "\"name\":\"" << name << "\"";
  oss << "}}";

  return oss.str();
}

std::string MvtSerializer::createNodeFeature(const std::pair<int32_t, int32_t>& coords,
                                            const valhalla::baldr::NodeInfo* node) {
  std::ostringstream oss;

  oss << "{\"type\":\"Feature\",\"geometry\":{";
  oss << "\"type\":\"Point\",\"coordinates\":[" << coords.first << "," << coords.second << "]";
  oss << "},\"properties\":{";
  auto point = node->latlng(valhalla::midgard::PointLL(0, 0)); // Use dummy tile corner for now
  oss << "\"node_id\":" << point.lat() << ",";
  oss << "\"type\":\"intersection\"";
  oss << "}}";

  return oss.str();
}

valhalla::midgard::AABB2<valhalla::midgard::PointLL> MvtSerializer::calculateTileBounds(uint32_t z, uint32_t x, uint32_t y) {
    // Convert tile coordinates to lat/lng bounds using standard Web Mercator tiling
  // This follows the standard slippy map tile scheme used by most mapping services

  // Number of tiles at this zoom level
  uint32_t n = 1 << z;

  // Convert tile coordinates to longitude/latitude
  double lon_deg_per_tile = 360.0 / n;
  double lat_rad_per_tile = M_PI / n;

  double min_lon = x * lon_deg_per_tile - 180.0;
  double max_lon = (x + 1) * lon_deg_per_tile - 180.0;

  // Convert y to latitude using proper Web Mercator projection
  // y=0 is at the top (north), y=n-1 is at the bottom (south)
  double min_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * (y + 1.0) / n)));
  double max_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * y / n)));

  double min_lat = min_lat_rad * 180.0 / M_PI;
  double max_lat = max_lat_rad * 180.0 / M_PI;

  // Log the calculation details
  LOG_INFO("MVT DEBUG: Tile calculation for z=" + std::to_string(z) + " x=" + std::to_string(x) + " y=" + std::to_string(y));
  LOG_INFO("MVT DEBUG: n = 2^" + std::to_string(z) + " = " + std::to_string(n));
  LOG_INFO("MVT DEBUG: lon_deg_per_tile = 360.0 / " + std::to_string(n) + " = " + std::to_string(lon_deg_per_tile));
  LOG_INFO("MVT DEBUG: lat_rad_per_tile = π / " + std::to_string(n) + " = " + std::to_string(lat_rad_per_tile));
  LOG_INFO("MVT DEBUG: min_lon = " + std::to_string(x) + " * " + std::to_string(lon_deg_per_tile) + " - 180.0 = " + std::to_string(min_lon));
  LOG_INFO("MVT DEBUG: max_lon = (" + std::to_string(x) + " + 1) * " + std::to_string(lon_deg_per_tile) + " - 180.0 = " + std::to_string(max_lon));
  LOG_INFO("MVT DEBUG: min_lat_rad = atan(sinh(π * (1.0 - 2.0 * (" + std::to_string(y) + " + 1.0) / " + std::to_string(n) + "))) = " + std::to_string(min_lat_rad));
  LOG_INFO("MVT DEBUG: max_lat_rad = atan(sinh(π * (1.0 - 2.0 * " + std::to_string(y) + " / " + std::to_string(n) + "))) = " + std::to_string(max_lat_rad));
  LOG_INFO("MVT DEBUG: min_lat = " + std::to_string(min_lat_rad) + " * 180.0 / π = " + std::to_string(min_lat));
  LOG_INFO("MVT DEBUG: max_lat = " + std::to_string(max_lat_rad) + " * 180.0 / π = " + std::to_string(max_lat));
  LOG_INFO("MVT DEBUG: Final bounds: (" + std::to_string(min_lon) + ", " + std::to_string(min_lat) + ") to (" + std::to_string(max_lon) + ", " + std::to_string(max_lat) + ")");



  return valhalla::midgard::AABB2<valhalla::midgard::PointLL>(
    valhalla::midgard::PointLL(min_lon, min_lat),
    valhalla::midgard::PointLL(max_lon, max_lat)
  );
}

std::string MvtSerializer::generateMvtProtobuf(uint32_t z, uint32_t x, uint32_t y,
                                               const valhalla::midgard::AABB2<valhalla::midgard::PointLL>& bbox,
                                               const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader) {
  LOG_INFO("MVT DEBUG: Generating MVT protobuf data for zoom " + std::to_string(z));

    LOG_INFO("MVT DEBUG: Processing MVT tile " + std::to_string(z) + "/" + std::to_string(x) + "/" + std::to_string(y));

  try {
    // Create a buffer for the MVT data
    std::string buffer;

                        // Use vtzero for proper MVT generation
    LOG_INFO("MVT DEBUG: Implementing vtzero MVT encoder with Valhalla data");

          // Log the tile bounds for debugging
      LOG_INFO("MVT DEBUG: Calculated tile bounds: " +
               std::to_string(bbox.minx()) + "," + std::to_string(bbox.miny()) + " to " +
               std::to_string(bbox.maxx()) + "," + std::to_string(bbox.maxy()));

      // For NYC tile 14/4824/6159, expected bounds should be roughly:
      // Longitude: -74.3 to -74.2 (Manhattan area)
      // Latitude: 40.7 to 40.8 (Manhattan area)

    // TODO: In a full implementation, we would:
    // 1. Use GraphReader to access Valhalla graph tiles
    // 2. Iterate through edges and nodes in the tile bounds
    // 3. Convert lat/lng coordinates to tile-relative coordinates (0-4096)
    // 4. Create LineString features for roads and Point features for intersections
    // 5. Add road properties like speed, road class, names, etc.

        try {
      // Create a tile builder
      vtzero::tile_builder tile;

            // Create a layer for roads (extent defaults to 4096)
      vtzero::layer_builder layer{tile, "roads"};

      // No point features - only road segments
      // This will now only generate LineString features for roads
      // Either real Valhalla data or fallback grid roads

      // Road segments will be created below

      // Add some sample road features based on the tile bounds
      // These represent where we would place actual road data from Valhalla
      // Note: MVT coordinates are tile-relative (0-4096), not lat/lng
      // When we add real Valhalla data, we'll convert lat/lng to tile-relative coordinates

      // No point features - only LineString road segments will be created
      LOG_INFO("MVT DEBUG: Skipping point features, focusing on road segments only");

            // Add real Valhalla road data
      LOG_INFO("MVT DEBUG: Extracting real Valhalla road data");

      // Helper function to convert lat/lng to tile-relative coordinates
      auto convertToTileCoords = [&bbox](const valhalla::midgard::PointLL& latlng) -> std::pair<int32_t, int32_t> {
        double x = (latlng.lng() - bbox.minx()) / (bbox.maxx() - bbox.minx()) * 4096;
        // Note: MVT uses y=0 at the top, so we invert the y coordinate
        double y = (bbox.maxy() - latlng.lat()) / (bbox.maxy() - bbox.miny()) * 4096;

        // Clamp coordinates to tile bounds (0-4095)
        x = std::max(0.0, std::min(4095.0, x));
        y = std::max(0.0, std::min(4095.0, y));

        return {static_cast<int32_t>(x), static_cast<int32_t>(y)};
      };



      // TODO: Get GraphReader from context - for now we'll create sample road data
      // In a full implementation, we would:
      // 1. Get GraphReader from the calling context
      // 2. Find graph tiles that intersect with the tile bounds
      // 3. Iterate through edges in those tiles
      // 4. Extract edge geometry and properties

      // For now, let's create realistic road features based on the tile bounds
      // These represent where real roads would be placed

      // Example of how we would extract real Valhalla data:
      //
      // GraphReader reader;
      // std::vector<GraphId> tile_ids = reader.GetTileSet(bbox);
      //
      // for (const auto& tile_id : tile_ids) {
      //   auto tile = reader.GetGraphTile(tile_id);
      //   if (!tile) continue;
      //
      //   for (const auto& edge : tile->GetDirectedEdges()) {
      //     // Get edge geometry
      //     auto shape = tile->edgeinfo(edge.edgeinfo_offset()).shape();
      //
      //     // Convert coordinates to tile-relative
      //     std::vector<std::pair<int32_t, int32_t>> tile_coords;
      //     for (const auto& point : shape) {
      //       tile_coords.push_back(convertToTileCoords(point));
      //     }
      //
      //     // Create LineString feature
      //     vtzero::linestring_feature_builder road{layer};
      //     road.set_id(edge.edgeinfo_offset());
      //     road.add_linestring(tile_coords.size());
      //     for (const auto& coord : tile_coords) {
      //       road.set_point(coord.first, coord.second);
      //     }
      //     road.add_property("road_class", std::to_string(edge.road_class()));
      //     road.add_property("speed", std::to_string(edge.speed()));
      //     road.commit();
      //   }
      // }

      // Extract real Valhalla road data from graph tiles
      LOG_INFO("MVT DEBUG: Reading real Valhalla graph tiles");

      // Get the graph reader from the global context
      // Note: In a full implementation, this would be passed in from the calling context
      // For now, we'll create a temporary reader to demonstrate the concept

                  try {
        // Use provided GraphReader or create a new one if not provided
        std::shared_ptr<valhalla::baldr::GraphReader> reader;
        if (graph_reader) {
          LOG_INFO("MVT DEBUG: Using provided GraphReader");
          reader = graph_reader;
        } else {
          LOG_INFO("MVT DEBUG: Creating new GraphReader for real Valhalla data");
          // Try to create GraphReader with default configuration
          // The mjolnir section should contain the tile_dir path
          boost::property_tree::ptree config;
          try {
            // Try to read the Valhalla config file
            boost::property_tree::read_json("valhalla.json", config);
            LOG_INFO("MVT DEBUG: Successfully read valhalla.json config");
          } catch (const std::exception& e) {
            LOG_INFO("MVT DEBUG: Could not read valhalla.json, using default config");
            // Set default tile directory
            config.put("mjolnir.tile_dir", "/usr/local/share/valhalla/valhalla_tiles");
          }
          // Create GraphReader with the configuration
          reader = std::make_shared<valhalla::baldr::GraphReader>(config.get_child("mjolnir"));
          LOG_INFO("MVT DEBUG: Successfully created GraphReader");
        }

        // Use efficient spatial binning to get only edges that intersect with our bounding box
        LOG_INFO("MVT DEBUG: Using efficient edges_in_bbox to find relevant edges");
        auto edge_ids = loki::edges_in_bbox(bbox, *reader);
        LOG_INFO("MVT DEBUG: Found " + std::to_string(edge_ids.size()) + " edges in bounding box");

        uint32_t feature_id = 1; // Start with ID 1 for road segments
        uint32_t roads_found = 0;
        uint32_t edges_processed = 0;

        // Process only the edges that intersect with our bounding box
        for (const auto& edge_id : edge_ids) {
          // Get the tile for this edge
          auto tile = reader->GetGraphTile(edge_id);
          if (!tile) continue;

          // Get the edge from the tile
          const auto* edge = tile->directededge(edge_id);
          if (!edge) continue;

          // Get edge info for geometry and properties
          const auto& edge_info = tile->edgeinfo(edge);
          auto shape = edge_info.shape();

          if (shape.empty()) {
            continue;
          }

          // Convert edge coordinates to tile-relative and filter duplicates/zero-length segments
          std::vector<std::pair<int32_t, int32_t>> tile_coords;
          std::pair<int32_t, int32_t> last_coord = {-1, -1};
          bool edge_in_bounds = false;

          for (const auto& point : shape) {
            auto coord = convertToTileCoords(point);

            // Check if this coordinate is within the MVT tile bounds (0-4096)
            if (coord.first >= 0 && coord.first <= 4096 && coord.second >= 0 && coord.second <= 4096) {
              edge_in_bounds = true;
            }

            // Skip duplicate coordinates (zero-length segments)
            if (coord != last_coord) {
              tile_coords.push_back(coord);
              last_coord = coord;
            }
          }

          // Skip if edge is completely outside the MVT tile bounds
          if (!edge_in_bounds) {
            continue;
          }

          // Skip if we don't have enough points for a valid linestring
          if (tile_coords.size() < 2) {
            continue;
          }

          // For zoom levels < 12, only show class 0 (motorway) and class 1 (trunk) roads
          if (z < 12) {
            auto road_class = static_cast<int>(edge->classification());
            if (road_class != 0 && road_class != 1) {
              continue; // Skip this road
            }
          }

          if (z < 7) {
            auto road_class = static_cast<int>(edge->classification());
            if (road_class != 0) {
              continue; // Skip this road
            }
          }

          // Create individual MVT feature for this road (no merging for now)
          vtzero::linestring_feature_builder road{layer};
          road.set_id(feature_id++);
          road.add_linestring(tile_coords.size());

          for (const auto& coord : tile_coords) {
            road.set_point(coord.first, coord.second);
          }

          // Add road properties
          // road.add_property("type", "road");
          road.add_property("classification", std::to_string(static_cast<int>(edge->classification())));
          // road.add_property("speed", std::to_string(edge->speed()));
          // road.add_property("use", std::to_string(static_cast<int>(edge->use())));

          // Add road name if available
          auto names = edge_info.GetNames();
          if (!names.empty()) {
            // road.add_property("name", names[0]);
          }

          road.commit();
          roads_found++;
          edges_processed++;

          // Limit the number of roads to avoid overwhelming the tile
          if (roads_found >= 500000) {
            LOG_INFO("MVT DEBUG: Reached road limit (500000), stopping extraction");
            break;
          }
        }

        LOG_INFO("MVT DEBUG: Processed " + std::to_string(edges_processed) + " edges, found " + std::to_string(roads_found) + " roads");

      } catch (const std::exception& e) {
        LOG_ERROR("MVT DEBUG: Error reading Valhalla graph data: " + std::string(e.what()));
        // Re-throw the exception to let the caller handle it
        throw;
      }

      // Serialize the tile
      const auto data = tile.serialize();

      // Convert to string
      std::string buffer(data.data(), data.size());

      LOG_INFO("MVT DEBUG: vtzero MVT tile created, size: " + std::to_string(buffer.size()) + " bytes");

      return buffer;

    } catch (const std::exception& e) {
      LOG_ERROR("MVT DEBUG: Exception in vtzero MVT generation: " + std::string(e.what()));
      return "MVT_ERROR: vtzero generation failed - " + std::string(e.what());
    } catch (...) {
      LOG_ERROR("MVT DEBUG: Unknown exception in vtzero MVT generation");
      return "MVT_ERROR: vtzero generation failed - unknown exception";
    }

  } catch (const std::exception& e) {
    LOG_ERROR("MVT DEBUG: Exception in generateMvtProtobuf: " + std::string(e.what()));
    return "MVT_ERROR: " + std::string(e.what());
  } catch (...) {
    LOG_ERROR("MVT DEBUG: Unknown exception in generateMvtProtobuf");
    return "MVT_ERROR: Unknown exception";
  }
}

} // namespace tyr
} // namespace valhalla
