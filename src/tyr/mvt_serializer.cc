#include "tyr/mvt_serializer.h"
#include "proto/api.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/nodeinfo.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/time_info.h"
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

// --- Web Mercator conversion helpers ---
inline double lonToWorldX(double lon) {
  return (lon + 180.0) / 360.0; // normalized 0..1
}

inline double latToWorldY(double lat) {
  double rad = lat * M_PI / 180.0;
  double merc = std::log(std::tan(M_PI/4.0 + rad/2.0));
  return (1.0 - merc / M_PI) / 2.0; // normalized 0..1
}

// Convert a lat/lng to tile-local coordinates in [0,4095]
inline std::pair<double,double> projectToTile(
  const valhalla::midgard::PointLL& ll, uint32_t z, uint32_t x, uint32_t y)
{
  uint32_t n = 1 << z;
  double worldX = lonToWorldX(ll.lng());
  double worldY = latToWorldY(ll.lat());

  double scale = 4096.0; // MVT extent
  double tileX = (worldX * n - x) * scale;
  double tileY = (worldY * n - y) * scale;
  return {tileX, tileY};
}

// Build a tile-local clipped LineString
std::vector<std::pair<int32_t,int32_t>> buildClippedLineString(
  const std::vector<valhalla::midgard::PointLL>& coords, uint32_t z, uint32_t x, uint32_t y)
{
  std::vector<std::pair<int32_t,int32_t>> result;

  for (size_t i = 1; i < coords.size(); ++i) {
    auto [x0f, y0f] = projectToTile(coords[i-1], z, x, y);
    auto [x1f, y1f] = projectToTile(coords[i],   z, x, y);

    int32_t x0 = static_cast<int32_t>(std::round(x0f));
    int32_t y0 = static_cast<int32_t>(std::round(y0f));
    int32_t x1 = static_cast<int32_t>(std::round(x1f));
    int32_t y1 = static_cast<int32_t>(std::round(y1f));

    // If both inside and not zero-length
    if (x0 >= 0 && x0 <= 4095 && y0 >= 0 && y0 <= 4095 &&
        x1 >= 0 && x1 <= 4095 && y1 >= 0 && y1 <= 4095) {
      if (x0 != x1 || y0 != y1) {
        if (result.empty()) result.emplace_back(x0, y0);
        result.emplace_back(x1, y1);
      }
    } else {
      // Clip against tile bounds
      if (x0f == x1f && y0f == y1f) continue; // skip zero-length
      auto clipped = clipSegment(x0f, y0f, x1f, y1f);
      if (!clipped.empty() && clipped.size() >= 2) {
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

// Helper function to encode a single polyline value
std::string encodePolylineValue(int32_t value) {
  // Left shift by 1 bit
  value = value << 1;

  // If the original value is negative, invert
  if (value < 0) {
    value = ~value;
  }

  std::string result = "";
  while (value >= 0x20) {
    result += static_cast<char>((0x20 | (value & 0x1f)) + 63);
    value >>= 5;
  }
  result += static_cast<char>(value + 63);
  return result;
}

// Helper function to simplify geometry based on zoom level
std::vector<std::pair<int32_t, int32_t>> simplifyGeometry(
    const std::vector<std::pair<int32_t, int32_t>>& coords, uint32_t zoom) {

  if (coords.size() <= 2) {
    return coords; // Can't simplify with less than 3 points
  }

  uint32_t step = 1;
  if (zoom <= 6) {
    step = 4; // Keep every 4th point at zoom 5-6
  } else if (zoom <= 8) {
    step = 3; // Keep every 3rd point at zoom 7-8
  } else if (zoom <= 10) {
    step = 2; // Keep every 2nd point at zoom 9-10
  }

  std::vector<std::pair<int32_t, int32_t>> simplified;
  simplified.reserve((coords.size() + step - 1) / step);

  simplified.push_back(coords[0]);

  for (size_t i = step; i < coords.size() - 1; i += step) {
    if (coords[i] != simplified.back()) {
      simplified.push_back(coords[i]);
    }
  }

  // Always keep the last point (unless it's the same as the last added point)
  if (coords.size() > 1 && coords.back() != simplified.back()) {
    simplified.push_back(coords.back());
  }

  // Ensure we have at least 2 different points
  if (simplified.size() < 2) {
    return coords;
  }

  // Remove any consecutive duplicate points that might have been created
  std::vector<std::pair<int32_t, int32_t>> final_coords;
  final_coords.reserve(simplified.size());
  final_coords.push_back(simplified[0]);

  for (size_t i = 1; i < simplified.size(); ++i) {
    if (simplified[i] != final_coords.back()) {
      final_coords.push_back(simplified[i]);
    }
  }

  // Ensure we still have at least 2 points after deduplication
  if (final_coords.size() < 2) {
    return coords; // Return original if deduplication removed too many points
  }

  return final_coords;
}

namespace valhalla {
namespace tyr {


std::string MvtSerializer::serialize(const valhalla::Api& api, const valhalla::Options_Format& format,
                                    const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader,
                                    const boost::property_tree::ptree* config) {

  // Currently only support MVT format
  if (format != valhalla::Options_Format_mvt) {
    LOG_ERROR("MVT DEBUG: Unsupported format: " + std::to_string(format));
    throw std::runtime_error("MVT serialization requires MVT format");
  }

  // Parse tile coordinates from the API options id field
  // The id field contains "z/x/y" coordinates from the tile endpoint
  std::string tile_id = api.options().id();
  LOG_INFO("MVT DEBUG: Tile ID from API: " + tile_id);

  if (tile_id.empty() || tile_id == "tile_endpoint_active") {
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

    // Calculate tile bounds (will be used for actual tile generation later)
    auto bbox = calculateTileBounds(z, x, y);

        std::string mvt_data = generateMvtProtobuf(z, x, y, bbox, graph_reader, config, api);
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

  double min_lon = x * lon_deg_per_tile - 180.0;
  double max_lon = (x + 1) * lon_deg_per_tile - 180.0;

  // Convert y to latitude using proper Web Mercator projection
  // y=0 is at the top (north), y=n-1 is at the bottom (south)
  // Web Mercator has a maximum latitude of approximately ±85.0511 degrees
  double max_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * y / n)));
  double min_lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * (y + 1.0) / n)));

  double max_lat = max_lat_rad * 180.0 / M_PI;
  double min_lat = min_lat_rad * 180.0 / M_PI;

  // Log tile bounds for debugging
  LOG_INFO("MVT DEBUG: Tile bounds z=" + std::to_string(z) + " x=" + std::to_string(x) + " y=" + std::to_string(y) +
           " -> lon[" + std::to_string(min_lon) + "," + std::to_string(max_lon) + "] lat[" + std::to_string(min_lat) + "," + std::to_string(max_lat) + "]");

  return valhalla::midgard::AABB2<valhalla::midgard::PointLL>(
    valhalla::midgard::PointLL(min_lon, min_lat),
    valhalla::midgard::PointLL(max_lon, max_lat)
  );
}

std::string MvtSerializer::generateMvtProtobuf(uint32_t z, uint32_t x, uint32_t y,
                                               const valhalla::midgard::AABB2<valhalla::midgard::PointLL>& bbox,
                                               const std::shared_ptr<valhalla::baldr::GraphReader>& graph_reader,
                                               const boost::property_tree::ptree* config,
                                               const valhalla::Api& api) {

    LOG_INFO("MVT DEBUG: Processing MVT tile " + std::to_string(z) + "/" + std::to_string(x) + "/" + std::to_string(y));

  try {
    // Create a buffer for the MVT data
    std::string buffer;

        try {
      // Create a tile builder
      vtzero::tile_builder tile;

      // Create a layer for roads (extent defaults to 4096)
      vtzero::layer_builder layer{tile, "traffic"};


        try {
        std::shared_ptr<valhalla::baldr::GraphReader> reader;
        if (!graph_reader) {
          return "No graph reader provided";
        }

        // Determine which Valhalla tile levels to include based on zoom level
        std::vector<uint8_t> allowed_tile_levels;
        LOG_INFO("MVT DEBUG: Config pointer: " + std::to_string(reinterpret_cast<uintptr_t>(config)));
        if (config) {
          // Use configuration-based filtering
          uint32_t tile_0_min_zoom = config->get("map_tile.valhalla_tile_0_min_zoom", 5);
          uint32_t tile_1_min_zoom = config->get("map_tile.valhalla_tile_1_min_zoom", 12);
          uint32_t tile_2_min_zoom = config->get("map_tile.valhalla_tile_2_min_zoom", 16);

          if (z >= tile_0_min_zoom) allowed_tile_levels.push_back(0);
          if (z >= tile_1_min_zoom) allowed_tile_levels.push_back(1);
          if (z >= tile_2_min_zoom) allowed_tile_levels.push_back(2);
        }

        auto edge_ids = loki::edges_in_bbox(bbox, *graph_reader);

        uint32_t roads_found = 0;
        uint32_t edges_processed = 0;

        // Process only the edges that intersect with our bounding box
        for (const auto& edge_id : edge_ids) {
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
          auto tile = graph_reader->GetGraphTile(edge_id);
          if (!tile) continue;

          // Get the edge from the tile
          const auto* edge = tile->directededge(edge_id);
          if (!edge) continue;

          // Get the opposing edge (backward direction)
          graph_tile_ptr opp_tile = nullptr;
          const auto* opp_edge = graph_reader->GetOpposingEdge(edge_id, opp_tile);

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

          std::vector<std::pair<int32_t, int32_t>> filtered_coords;
          filtered_coords.reserve(tile_coords.size());

          for (size_t i = 0; i < tile_coords.size(); ++i) {
            if (i == 0 || tile_coords[i] != tile_coords[i-1]) {
              filtered_coords.push_back(tile_coords[i]);
            }
          }

          if (filtered_coords.size() < 2) {
            continue;
          }

          // Filter by road class based on zoom level
          auto road_class = static_cast<int>(edge->classification());
          bool road_class_allowed = false;

          if (config) {
            // Use configuration-based road class filtering
            uint32_t class_0_min_zoom = config->get("map_tile.valhalla_road_class_0_min_zoom", 5);
            uint32_t class_1_min_zoom = config->get("map_tile.valhalla_road_class_1_min_zoom", 7);
            uint32_t class_2_min_zoom = config->get("map_tile.valhalla_road_class_2_min_zoom", 12);
            uint32_t class_3_min_zoom = config->get("map_tile.valhalla_road_class_3_min_zoom", 13);
            uint32_t class_4_min_zoom = config->get("map_tile.valhalla_road_class_4_min_zoom", 14);
            uint32_t all_classes_min_zoom = config->get("map_tile.valhalla_all_road_classes_min_zoom", 14);

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
          }

          if (!road_class_allowed) {
            continue; // Skip this road class at this zoom level
          }

          if (edge->is_shortcut() && z < 13) {
            continue; // Skip shortcut edges at high zoom levels
          }

          if (edge->use() == baldr::Use::kFerry) {
            continue; // Skip ferry edges
          }

          // Filter out short edges at low zoom levels (5 and 6)
          if ((z == 5 || z == 6) && edge->length() < 500) {
            continue; // Skip edges shorter than 500 meters at zoom 5-6
          }

          // Simplify geometry based on zoom level to reduce coordinate density
          auto simplified_coords = simplifyGeometry(filtered_coords, z);

          // Skip if simplification resulted in insufficient points
          if (simplified_coords.size() < 2) {
            continue;
          }

          bool simplified_has_zero_length = true;
          for (size_t i = 1; i < simplified_coords.size(); ++i) {
            if (simplified_coords[i] != simplified_coords[0]) {
              simplified_has_zero_length = false;
              break;
            }
          }
          if (simplified_has_zero_length) {
            continue;
          }

          // Create individual MVT feature for this road (no merging for now)
          vtzero::linestring_feature_builder road{layer};
          road.set_id(edge_id.value);
          road.add_linestring(simplified_coords.size());

          for (const auto& coord : simplified_coords) {
            road.set_point(coord.first, coord.second);
          }

          // Add basic road properties
          road.add_property("classification", std::to_string(static_cast<int>(edge->classification())));
          road.add_property("id", static_cast<int64_t>(edge_id.value));
          road.add_property("is_shortcut", edge->is_shortcut());

          // Add access restrictions (height, width, length, weight only)
          if (edge->access_restriction()) {
            // Get access restrictions for this edge
            auto restrictions = tile->GetAccessRestrictions(edge_id.id(), baldr::kAllAccess);

            for (const auto& restriction : restrictions) {
              switch (restriction.type()) {
                case baldr::AccessType::kMaxHeight:
                  road.add_property("max_height", static_cast<int64_t>(restriction.value()));
                  break;
                case baldr::AccessType::kMaxWidth:
                  road.add_property("max_width", static_cast<int64_t>(restriction.value()));
                  break;
                case baldr::AccessType::kMaxLength:
                  road.add_property("max_length", static_cast<int64_t>(restriction.value()));
                  break;
                case baldr::AccessType::kMaxWeight:
                  road.add_property("max_weight", static_cast<int64_t>(restriction.value()));
                  break;
                default:
                  break;
              }
            }
          }

          if (edge->has_flow_speed()) {
            if (edge->free_flow_speed() > 0) {
              road.add_property("free_flow_speed", static_cast<int64_t>(edge->free_flow_speed()));
            }
          }

          // Get traffic speed (live or historic based on time parameter)
          if (tile->get_traffic_tile()()) {
            try {
              // Determine which traffic data to use based on API options
              uint64_t seconds_of_week = baldr::kInvalidSecondsOfWeek;
              uint64_t seconds_from_now = 0;

              // Check if a specific time was requested
              if (api.options().has_date_time_case()) {
                LOG_INFO("MVT DEBUG: Time parameter found: " + api.options().date_time());
                // Parse the requested time and convert to seconds of week for historic traffic
                try {
                  std::string date_time = api.options().date_time();
                  auto time_info = baldr::TimeInfo::make(date_time, 0, nullptr);
                  if (time_info.valid) {
                    seconds_of_week = time_info.second_of_week;
                    seconds_from_now = time_info.negative_seconds_from_now ?
                      -static_cast<int64_t>(time_info.seconds_from_now) :
                      static_cast<int64_t>(time_info.seconds_from_now);
                    LOG_INFO("MVT DEBUG: Parsed time - seconds_of_week: " + std::to_string(seconds_of_week) + ", seconds_from_now: " + std::to_string(seconds_from_now));
                  }
                } catch (const std::exception& e) {
                  // If time parsing fails, fall back to current traffic
                  LOG_DEBUG("Failed to parse time parameter, using current traffic: " + std::string(e.what()));
                }
              } else {
                LOG_INFO("MVT DEBUG: No time parameter found, using current traffic");
              }

              // Get both current and historic traffic speeds
              uint8_t current_flow_sources = 0;
              uint8_t historic_flow_sources = 0;

              // Always get current traffic speed (use current flow mask to get live traffic data)
              uint32_t current_speed = tile->GetSpeed(edge, baldr::kCurrentFlowMask, 0, false, &current_flow_sources);

              // Get historic traffic speed if time parameter is specified
              uint32_t historic_speed = 0;
              bool has_historic_data = false;
              if (api.options().has_date_time_case()) {
                historic_speed = tile->GetSpeed(edge, baldr::kPredictedFlowMask, seconds_of_week, false, &historic_flow_sources, seconds_from_now);
                has_historic_data = (historic_flow_sources & baldr::kPredictedFlowMask);
              }

              // Determine which speed to use as the primary "traffic_speed" property
              uint32_t primary_speed;

              if (api.options().has_date_time_case()) {
                // If time parameter is specified, use historic speed (with free flow fallback)
                primary_speed = has_historic_data ? historic_speed : edge->free_flow_speed();
              } else {
                // If no time parameter, use current speed (with free flow fallback)
                bool has_current_data = (current_flow_sources & baldr::kCurrentFlowMask);
                primary_speed = has_current_data ? current_speed : edge->free_flow_speed();
              }


              // Always add current speed (with free flow fallback if no current data)
              bool has_current_data = (current_flow_sources & baldr::kCurrentFlowMask);
              uint32_t current_speed_final = has_current_data ? current_speed : edge->free_flow_speed();
              road.add_property("current_speed", static_cast<int64_t>(current_speed_final));

              // Add historic speed if time parameter was specified
              if (api.options().has_date_time_case()) {
                uint32_t historic_speed_final = has_historic_data ? historic_speed : edge->free_flow_speed();
                road.add_property("historic_speed", static_cast<int64_t>(historic_speed_final));
              }

              // Calculate speed bucket based on traffic vs free flow speed
              if (edge->free_flow_speed() > 0) {
                double speed_ratio = static_cast<double>(primary_speed) / static_cast<double>(edge->free_flow_speed());
                int64_t speed_bucket = 0;

                if (speed_ratio == 0) {
                  speed_bucket = 0; // No traffic
                } else if (speed_ratio < 0.10) {
                  speed_bucket = 1; // Under 10% - Severe congestion
                } else if (speed_ratio < 0.25) {
                  speed_bucket = 2; // Under 25% - Heavy congestion
                } else if (speed_ratio < 0.60) {
                  speed_bucket = 3; // Under 65% - Moderate congestion
                } else {
                  speed_bucket = 4; // 100%+ - Free flow or better
                }

                road.add_property("speed_bucket", speed_bucket);
                road.add_property("speed_ratio", static_cast<double>(speed_ratio));
              }

              // Get predicted speed for current time (if available)
              // if (edge->has_predicted_speed()) {
              //   uint8_t predicted_flow_sources = 0;
              //   uint32_t predicted_speed = tile->GetSpeed(edge, baldr::kPredictedFlowMask, 0, false, &predicted_flow_sources);
              //   if (predicted_flow_sources & baldr::kPredictedFlowMask) {
              //     road.add_property("predicted_speed", static_cast<int64_t>(predicted_speed));
              //   }
              // }
            } catch (const std::exception& e) {
              // Traffic data might not be available for this edge, continue without it
              LOG_DEBUG("MVT DEBUG: Could not get traffic data for edge: " + std::string(e.what()));
            }
          }

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

}
}
