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
#include <optional>
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>

// protozero is required by vtzero
#include "third_party/protozero/include/protozero/pbf_reader.hpp"
// vtzero for MVT encoding
#include "third_party/vtzero/include/vtzero/builder.hpp"

// Bitmasks for Cohen–Sutherland line clipping
enum OutCode {
  INSIDE = 0, LEFT = 1, RIGHT = 2, BOTTOM = 4, TOP = 8
};

inline int computeOutCode(double x, double y, double minX, double minY, double maxX, double maxY) {
  int code = INSIDE;
  if (x < minX) code |= LEFT;
  else if (x > maxX) code |= RIGHT;
  if (y < minY) code |= TOP;       // Note: y=0 is top in MVT
  else if (y > maxY) code |= BOTTOM;
  return code;
}

// Clip a segment (in tile coords) to the tile box [0,4095]
std::vector<std::pair<int32_t,int32_t>> clipSegment(
  double x0, double y0, double x1, double y1,
  double minX=0.0, double minY=0.0, double maxX=4095.0, double maxY=4095.0)
{
  int outcode0 = computeOutCode(x0, y0, minX, minY, maxX, maxY);
  int outcode1 = computeOutCode(x1, y1, minX, minY, maxX, maxY);

  bool accept = false;
  while (true) {
    if (!(outcode0 | outcode1)) {
      accept = true; break; // both inside
    } else if (outcode0 & outcode1) {
      break; // both outside same edge
    } else {
      double x, y;
      int outcodeOut = outcode0 ? outcode0 : outcode1;

      if (outcodeOut & TOP) {
        x = x0 + (x1 - x0) * (minY - y0) / (y1 - y0);
        y = minY;
      } else if (outcodeOut & BOTTOM) {
        x = x0 + (x1 - x0) * (maxY - y0) / (y1 - y0);
        y = maxY;
      } else if (outcodeOut & RIGHT) {
        y = y0 + (y1 - y0) * (maxX - x0) / (x1 - x0);
        x = maxX;
      } else {
        y = y0 + (y1 - y0) * (minX - x0) / (x1 - x0);
        x = minX;
      }

      if (outcodeOut == outcode0) {
        x0 = x; y0 = y; outcode0 = computeOutCode(x0, y0, minX, minY, maxX, maxY);
      } else {
        x1 = x; y1 = y; outcode1 = computeOutCode(x1, y1, minX, minY, maxX, maxY);
      }
    }
  }

  std::vector<std::pair<int32_t,int32_t>> clipped;
  if (accept) {
    clipped.emplace_back(static_cast<int32_t>(std::round(x0)), static_cast<int32_t>(std::round(y0)));
    clipped.emplace_back(static_cast<int32_t>(std::round(x1)), static_cast<int32_t>(std::round(y1)));
  }
  return clipped;
}

// Build a tile-local clipped LineString
std::vector<std::pair<int32_t,int32_t>> buildClippedLineString(
  const std::vector<valhalla::midgard::PointLL>& coords, uint32_t z, uint32_t x, uint32_t y)
{
  std::vector<std::pair<int32_t,int32_t>> result;

  // Helper function to convert lat/lng to tile-relative coordinates using z,x,y directly
  auto convertToTileCoords = [z, x, y](const valhalla::midgard::PointLL& latlng) -> std::optional<std::pair<int32_t, int32_t>> {
    uint32_t n = 1 << z;
    double lon_deg_per_tile = 360.0 / n;

    double min_lon = x * lon_deg_per_tile - 180.0;
    double max_lon = (x + 1) * lon_deg_per_tile - 180.0;
    double x_ratio = (latlng.lng() - min_lon) / (max_lon - min_lon);

    double min_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * (y + 1.0) / n)));
    double max_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * y / n)));
    double min_lat = min_lat_rad * 180.0 / M_PI;
    double max_lat = max_lat_rad * 180.0 / M_PI;

    double y_ratio = (max_lat - latlng.lat()) / (max_lat - min_lat);

    if (x_ratio < 0.0 || x_ratio > 1.0 || y_ratio < 0.0 || y_ratio > 1.0) {
      return std::nullopt;
    }

    double tile_x = x_ratio * 4096.0;
    double tile_y = y_ratio * 4096.0;

    if (std::abs(x_ratio) < 1e-10) tile_x = 0.0;
    else if (std::abs(x_ratio - 1.0) < 1e-10) tile_x = 4095.0;

    if (std::abs(y_ratio) < 1e-10) tile_y = 0.0;
    else if (std::abs(y_ratio - 1.0) < 1e-10) tile_y = 4095.0;

    int32_t final_x = static_cast<int32_t>(std::round(tile_x));
    int32_t final_y = static_cast<int32_t>(std::round(tile_y));

    final_x = std::max(0, std::min(4095, final_x));
    final_y = std::max(0, std::min(4095, final_y));

    return {{final_x, final_y}};
  };

  for (size_t i = 1; i < coords.size(); ++i) {
    auto p0 = convertToTileCoords(coords[i-1]);
    auto p1 = convertToTileCoords(coords[i]);

    if (p0 && p1) {
      // Both inside - check for zero-length segment
      if (*p0 != *p1) {
        if (result.empty()) result.push_back(*p0);
        result.push_back(*p1);
      }
    } else {
      // At least one outside → try clipping in tile coordinates
      // First project raw coords into tile-relative floats
      auto toTileFloat = [&](const valhalla::midgard::PointLL& ll) {
        uint32_t n = 1 << z;
        double lon_deg_per_tile = 360.0 / n;
        double min_lon = x * lon_deg_per_tile - 180.0;
        double max_lon = (x + 1) * lon_deg_per_tile - 180.0;
        double x_ratio = (ll.lng() - min_lon) / (max_lon - min_lon);

        double min_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * (y + 1.0) / n)));
        double max_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * y / n)));
        double min_lat = min_lat_rad * 180.0 / M_PI;
        double max_lat = max_lat_rad * 180.0 / M_PI;

        double y_ratio = (max_lat - ll.lat()) / (max_lat - min_lat);

        return std::make_pair(x_ratio * 4096.0, y_ratio * 4096.0);
      };

      auto [x0, y0] = toTileFloat(coords[i-1]);
      auto [x1, y1] = toTileFloat(coords[i]);

      // Skip zero-length segments before clipping
      if (x0 == x1 && y0 == y1) {
        continue;
      }

      auto clipped = clipSegment(x0, y0, x1, y1);
      if (!clipped.empty() && clipped.size() >= 2) {
        // Check if clipped segment has zero length
        if (clipped[0] != clipped[1]) {
          if (result.empty() || result.back() != clipped.front()) {
            result.push_back(clipped.front());
          }
          result.push_back(clipped.back());
        }
      }
    }
  }

  return result;
}

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
                                    const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader,
                                    const boost::property_tree::ptree* config) {
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
      std::string mvt_data = generateMvtProtobuf(z, x, y, bbox, graph_reader, config);
      LOG_INFO("MVT DEBUG: Generated MVT protobuf, size: " + std::to_string(mvt_data.size()));
      return mvt_data;

  } catch (const std::exception& e) {
    LOG_ERROR("MVT DEBUG: Failed to parse tile coordinates: " + std::string(e.what()));
    return "Failed to parse tile coordinates";
  }
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
                                               const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader,
                                               const boost::property_tree::ptree* config) {
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



        try {
      // Create a tile builder
      vtzero::tile_builder tile;

            // Create a layer for roads (extent defaults to 4096)
      vtzero::layer_builder layer{tile, "roads"};

      // No point features - only LineString road segments will be created
      LOG_INFO("MVT DEBUG: Skipping point features, focusing on road segments only");

            // Add real Valhalla road data
      LOG_INFO("MVT DEBUG: Extracting real Valhalla road data");


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

        // Determine which Valhalla tile levels to include based on zoom level
        std::vector<uint8_t> allowed_tile_levels;
        if (config) {
          // Use configuration-based filtering
          uint32_t tile_0_min_zoom = config->get("map_tile.valhalla_tile_0_min_zoom", 5);
          uint32_t tile_1_min_zoom = config->get("map_tile.valhalla_tile_1_min_zoom", 14);
          uint32_t tile_2_min_zoom = config->get("map_tile.valhalla_tile_2_min_zoom", 16);

          if (z >= tile_0_min_zoom) allowed_tile_levels.push_back(0);
          if (z >= tile_1_min_zoom) allowed_tile_levels.push_back(1);
          if (z >= tile_2_min_zoom) allowed_tile_levels.push_back(2);
        } else {
          // Fallback to hardcoded values
          if (z >= 5) allowed_tile_levels.push_back(0);
          if (z >= 14) allowed_tile_levels.push_back(1);
          if (z >= 16) allowed_tile_levels.push_back(2);
        }

        LOG_INFO("MVT DEBUG: Zoom " + std::to_string(z) + " allows tile levels: ");
        for (auto level : allowed_tile_levels) {
          LOG_INFO("MVT DEBUG:   - Level " + std::to_string(level));
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
          // Filter by tile level first (most efficient)
          bool tile_level_allowed = false;
          for (auto allowed_level : allowed_tile_levels) {
            if (edge_id.level() == allowed_level) {
              tile_level_allowed = true;
              break;
            }
          }
          if (!tile_level_allowed) {
            continue; // Skip edges from disallowed tile levels
          }

          // Get the tile for this edge
          auto tile = reader->GetGraphTile(edge_id);
          if (!tile) continue;

          // Get the edge from the tile
          const auto* edge = tile->directededge(edge_id);
          if (!edge) continue;

          // Get the opposing edge (backward direction)
          graph_tile_ptr opp_tile = nullptr;
          const auto* opp_edge = reader->GetOpposingEdge(edge_id, opp_tile);

          if (opp_edge && opp_tile) {

            try {

              uint8_t flow_sources_forward = 0;
              uint32_t current_speed_forward = tile->GetSpeed(edge, baldr::kCurrentFlowMask, 0, false, &flow_sources_forward);

              uint8_t flow_sources_opp = 0;
              uint32_t current_speed_opp = opp_tile->GetSpeed(opp_edge, baldr::kCurrentFlowMask, 0, false, &flow_sources_opp);

              if (flow_sources_opp & baldr::kCurrentFlowMask && ((flow_sources_forward & baldr::kCurrentFlowMask && current_speed_opp < current_speed_forward) || !(flow_sources_forward & baldr::kCurrentFlowMask))) { // Use the slower of the two edges
                edge = opp_edge;
                tile = opp_tile;
              }

            } catch (const std::exception& e) {
              LOG_ERROR("MVT DEBUG: Could not get traffic data for edge: " + std::string(e.what()));
            }
          }

          // Get edge info for geometry and properties
          const auto& edge_info = tile->edgeinfo(edge);
          auto shape = edge_info.shape();

          if (shape.empty()) {
            continue;
          }

          // Use clipped line string to handle edges that cross tile boundaries
          auto tile_coords = buildClippedLineString(shape, z, x, y);

          // Skip if we don't have enough points for a valid linestring
          if (tile_coords.size() < 2) {
            continue;
          }

          // Filter out zero-length segments and duplicate consecutive points
          std::vector<std::pair<int32_t, int32_t>> filtered_coords;
          filtered_coords.reserve(tile_coords.size());

          for (size_t i = 0; i < tile_coords.size(); ++i) {
            if (i == 0 || tile_coords[i] != tile_coords[i-1]) {
              filtered_coords.push_back(tile_coords[i]);
            }
          }

          // Skip if we don't have enough points after filtering duplicates
          if (filtered_coords.size() < 2) {
            continue;
          }

          // Check for zero-length linestring (all points are the same)
          bool has_zero_length = true;
          for (size_t i = 1; i < filtered_coords.size(); ++i) {
            if (filtered_coords[i] != filtered_coords[0]) {
              has_zero_length = false;
              break;
            }
          }

          if (has_zero_length) {
            continue; // Skip zero-length linestrings
          }

          // Filter by road class based on zoom level
          auto road_class = static_cast<int>(edge->classification());
          bool road_class_allowed = false;

          if (config) {
            // Use configuration-based road class filtering
            uint32_t class_0_min_zoom = config->get("map_tile.valhalla_road_class_0_min_zoom", 5);
            uint32_t class_1_min_zoom = config->get("map_tile.valhalla_road_class_1_min_zoom", 7);
            uint32_t class_2_min_zoom = config->get("map_tile.valhalla_road_class_2_min_zoom", 12);
            uint32_t class_3_min_zoom = config->get("map_tile.valhalla_road_class_3_min_zoom", 12);
            uint32_t class_4_min_zoom = config->get("map_tile.valhalla_road_class_4_min_zoom", 12);
            uint32_t all_classes_min_zoom = config->get("map_tile.valhalla_all_road_classes_min_zoom", 12);

            if (z >= all_classes_min_zoom) {
              road_class_allowed = true; // All road classes allowed
            } else if (road_class == 0 && z >= class_0_min_zoom) {
              road_class_allowed = true; // Motorway
            } else if (road_class == 1 && z >= class_1_min_zoom) {
              road_class_allowed = true; // Trunk
            } else if (road_class == 2 && z >= class_2_min_zoom) {
              road_class_allowed = true; // Primary
            } else if (road_class == 3 && z >= class_3_min_zoom) {
              road_class_allowed = true; // Secondary
            } else if (road_class == 4 && z >= class_4_min_zoom) {
              road_class_allowed = true; // Tertiary
            }
          } else {
            // Fallback to hardcoded values (matching the default configuration)
            if (z >= 12) {
              road_class_allowed = true; // All road classes at zoom 12+
            } else if (road_class == 0 && z >= 5) {
              road_class_allowed = true; // Motorway at zoom 5+
            } else if (road_class == 1 && z >= 7) {
              road_class_allowed = true; // Trunk at zoom 7+
            } else if (road_class == 2 && z >= 12) {
              road_class_allowed = true; // Primary at zoom 12+
            } else if (road_class == 3 && z >= 12) {
              road_class_allowed = true; // Secondary at zoom 12+
            } else if (road_class == 4 && z >= 12) {
              road_class_allowed = true; // Tertiary at zoom 12+
            }
          }

          if (!road_class_allowed) {
            continue; // Skip this road class at this zoom level
          }

          // Create individual MVT feature for this road (no merging for now)
          vtzero::linestring_feature_builder road{layer};
          road.set_id(feature_id++);
          road.add_linestring(filtered_coords.size());

          for (const auto& coord : filtered_coords) {
            road.set_point(coord.first, coord.second);
          }

          // Add basic road properties
          road.add_property("classification", std::to_string(static_cast<int>(edge->classification())));
          road.add_property("speed", static_cast<int64_t>(edge->speed()));
          road.add_property("length", static_cast<int64_t>(edge->length()));
          road.add_property("surface", static_cast<int64_t>(edge->surface()));
          road.add_property("use", static_cast<int64_t>(edge->use()));
          road.add_property("toll", edge->toll());
          road.add_property("roundabout", edge->roundabout());
          road.add_property("lanecount", static_cast<int64_t>(edge->lanecount()));
          road.add_property("density", static_cast<int64_t>(edge->density()));

          // Add traffic-related properties
          if (edge->has_flow_speed()) {
            // Free flow speed (nighttime/no traffic)
            if (edge->free_flow_speed() > 0) {
              road.add_property("free_flow_speed", static_cast<int64_t>(edge->free_flow_speed()));
            }

            // Constrained flow speed (daytime/traffic)
            if (edge->constrained_flow_speed() > 0) {
              road.add_property("constrained_flow_speed", static_cast<int64_t>(edge->constrained_flow_speed()));
            }

            // Predicted speed flag
            if (edge->has_predicted_speed()) {
              road.add_property("has_predicted_speed", true);
            }
          }

          // Get live traffic speed if available
          if (tile->get_traffic_tile()()) {
            try {
              // Get current live traffic speed
              uint8_t flow_sources = 0;
              uint32_t current_speed = tile->GetSpeed(edge, baldr::kCurrentFlowMask, 0, false, &flow_sources);
              if (flow_sources & baldr::kCurrentFlowMask) {
                road.add_property("current_traffic_speed", static_cast<int64_t>(current_speed));
                road.add_property("has_live_traffic", true);

                // Calculate speed bucket based on live traffic vs free flow speed
                if (edge->free_flow_speed() > 0) {
                  double speed_ratio = static_cast<double>(current_speed) / static_cast<double>(edge->free_flow_speed());
                  int64_t speed_bucket = 0;

                  if (speed_ratio < 0.10) {
                    speed_bucket = 1; // Under 10% - Severe congestion
                  } else if (speed_ratio < 0.25) {
                    speed_bucket = 2; // Under 25% - Heavy congestion
                  } else if (speed_ratio < 0.60) {
                    speed_bucket = 3; // Under 65% - Moderate congestion
                  } else if (speed_ratio < 1.00) {
                    speed_bucket = 4; // Under 100% - Light congestion
                  } else {
                    speed_bucket = 5; // 100%+ - Free flow or better
                  }

                  road.add_property("speed_bucket", speed_bucket);
                  road.add_property("speed_ratio", static_cast<double>(speed_ratio));
                }
              }

              // Get predicted speed for current time (if available)
              if (edge->has_predicted_speed()) {
                uint32_t predicted_speed = tile->GetSpeed(edge, baldr::kPredictedFlowMask, 0, false, &flow_sources);
                if (flow_sources & baldr::kPredictedFlowMask) {
                  road.add_property("predicted_speed", static_cast<int64_t>(predicted_speed));
                }
              }
            } catch (const std::exception& e) {
              // Traffic data might not be available for this edge, continue without it
              LOG_DEBUG("MVT DEBUG: Could not get traffic data for edge: " + std::string(e.what()));
            }
          }

          // Add road name if available
          // auto names = edge_info.GetNames();
          // if (!names.empty()) {
          //   road.add_property("name", names[0]);
          // }

          road.commit();
          roads_found++;
          edges_processed++;

          // Limit the number of roads to avoid overwhelming the tile
          if (roads_found >= 1000000) {
            LOG_INFO("MVT DEBUG: Reached road limit (1000000), stopping extraction");
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
