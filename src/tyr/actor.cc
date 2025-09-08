#include "tyr/actor.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include "tyr/mvt_serializer.h"

using namespace valhalla;
using namespace valhalla::loki;
using namespace valhalla::thor;
using namespace valhalla::odin;

namespace valhalla {
namespace tyr {

struct actor_t::pimpl_t {
  pimpl_t(const boost::property_tree::ptree& config)
      : reader(new baldr::GraphReader(config.get_child("mjolnir"))), loki_worker(config, reader),
        thor_worker(config, reader), odin_worker(config) {
  }
  pimpl_t(const boost::property_tree::ptree& config, baldr::GraphReader& graph_reader)
      : reader(&graph_reader, [](baldr::GraphReader*) {}), loki_worker(config, reader),
        thor_worker(config, reader), odin_worker(config) {
  }
  void set_interrupts(const std::function<void()>* interrupt_function) {
    loki_worker.set_interrupt(interrupt_function);
    thor_worker.set_interrupt(interrupt_function);
    odin_worker.set_interrupt(interrupt_function);
  }
  void cleanup() {
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
  }
  std::shared_ptr<baldr::GraphReader> reader;
  loki::loki_worker_t loki_worker;
  thor::thor_worker_t thor_worker;
  odin_worker_t odin_worker;
};

actor_t::actor_t(const boost::property_tree::ptree& config, bool auto_cleanup)
    : pimpl(new pimpl_t(config)), auto_cleanup(auto_cleanup) {
}
actor_t::actor_t(const boost::property_tree::ptree& config,
                 baldr::GraphReader& reader,
                 bool auto_cleanup)
    : pimpl(new pimpl_t(config, reader)), auto_cleanup(auto_cleanup) {
}

void actor_t::cleanup() {
  pimpl->cleanup();
}

std::string actor_t::act(Api& api, const std::function<void()>* interrupt) {
  if (api.options().action() == Options::no_action)
    throw valhalla_exception_t{106};

  switch (api.options().action()) {
    case Options::route:
      return route("", interrupt, &api);
    case Options::locate:
      return locate("", interrupt, &api);
    case Options::sources_to_targets:
      return matrix("", interrupt, &api);
    case Options::optimized_route:
      return optimized_route("", interrupt, &api);
    case Options::isochrone:
      return isochrone("", interrupt, &api);
    case Options::trace_route:
      return trace_route("", interrupt, &api);
    case Options::trace_attributes:
      return trace_attributes("", interrupt, &api);
    case Options::height:
      return height("", interrupt, &api);
    case Options::transit_available:
      return transit_available("", interrupt, &api);
    case Options::expansion:
      return expansion("", interrupt, &api);
    case Options::centroid:
      return centroid("", interrupt, &api);
    case Options::status:
      return status("", interrupt, &api);
      case Options::tile:
        LOG_INFO("ACTOR DEBUG: Handling tile action");
        LOG_INFO("ACTOR DEBUG: About to call serializeMvt");
        try {
          auto response = serializeMvt(api, pimpl->reader);
          LOG_INFO("ACTOR DEBUG: serializeMvt completed successfully, response size: " + std::to_string(response.size()));
          return response;
        } catch (const std::exception& e) {
          LOG_ERROR("ACTOR DEBUG: serializeMvt failed with error: " + std::string(e.what()));
          throw;
        }
    default:
      throw valhalla_exception_t{106};
  }
}

std::string
actor_t::route(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::route, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.route(*api);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.route(*api);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string
actor_t::locate(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::locate, *api);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.locate(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string
actor_t::matrix(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::sources_to_targets, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(*api);
  // compute the matrix
  auto bytes = pimpl->thor_worker.matrix(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string actor_t::optimized_route(const std::string& request_str,
                                     const std::function<void()>* interrupt,
                                     Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::optimized_route, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(*api);
  // compute compute all pairs and then the shortest path through them all
  pimpl->thor_worker.optimized_route(*api);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string
actor_t::isochrone(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::isochrone, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.isochrones(*api);
  // compute the isochrones
  auto json = pimpl->thor_worker.isochrones(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string actor_t::trace_route(const std::string& request_str,
                                 const std::function<void()>* interrupt,
                                 Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::trace_route, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(*api);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.trace_route(*api);
  // get some directions back from them
  auto bytes = pimpl->odin_worker.narrate(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string actor_t::trace_attributes(const std::string& request_str,
                                      const std::function<void()>* interrupt,
                                      Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::trace_attributes, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(*api);
  // get the path and turn it into attribution along it
  auto json = pimpl->thor_worker.trace_attributes(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string
actor_t::height(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::height, *api);
  // get the height at each point
  auto json = pimpl->loki_worker.height(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string actor_t::transit_available(const std::string& request_str,
                                       const std::function<void()>* interrupt,
                                       Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::transit_available, *api);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.transit_available(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string
actor_t::expansion(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::expansion, *api);
  // check the request and locate the locations in the graph
  if (api->options().expansion_action() == Options::route) {
    pimpl->loki_worker.route(*api);
  } else if (api->options().expansion_action() == Options::isochrone) {
    pimpl->loki_worker.isochrones(*api);
  } else {
    pimpl->loki_worker.matrix(*api);
  }
  // route between the locations in the graph to find the best path
  auto json = pimpl->thor_worker.expansion(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string
actor_t::centroid(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::centroid, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.route(*api);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.centroid(*api);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string
actor_t::status(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::status, *api);
  // check lokis status
  pimpl->loki_worker.status(*api);
  // check thors status
  pimpl->thor_worker.status(*api);
  // check odins status
  pimpl->odin_worker.status(*api);
  // get the json
  auto json = tyr::serializeStatus(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string
actor_t::tile(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::tile, *api);

  // Check if we have tile coordinates in the id field (from HTTP route parsing)
  if (api->options().has_id()) {
    std::string tile_id = api->options().id();

    // Parse z/x/y from the id field (format: "z/x/y")
    std::vector<std::string> parts;
    std::stringstream ss(tile_id);
    std::string part;
    while (std::getline(ss, part, '/')) {
      parts.push_back(part);
    }

    if (parts.size() == 3) {
      try {
        uint32_t z = std::stoul(parts[0]);
        uint32_t x = std::stoul(parts[1]);
        uint32_t y = std::stoul(parts[2]);

        // Use the tile_xyz function for proper tile generation
        auto mvt_data = tile_xyz(z, x, y, interrupt);
        return mvt_data;

      } catch (const std::exception& e) {
        throw valhalla_exception_t{400, "Invalid tile coordinates: " + tile_id};
      }
    }
  }

  // Fallback: Check if we have locations to define the bounding box (for testing)
  if (api->options().locations_size() >= 2) {
    // For now, use locations to create a bounding box
    // But this should be replaced with proper tile coordinate handling
    auto point1 = api->options().locations(0).ll();
    auto point2 = api->options().locations(1).ll();

    // Create bounding box with min/max coordinates
    double min_lat = std::min(point1.lat(), point2.lat());
    double max_lat = std::max(point1.lat(), point2.lat());
    double min_lng = std::min(point1.lng(), point2.lng());
    double max_lng = std::max(point1.lng(), point2.lng());

    auto bbox = midgard::AABB2<midgard::PointLL>(min_lng, min_lat, max_lng, max_lat);

    // Default zoom level
    uint32_t z = 14;

    // Generate MVT tile
    auto mvt_data = tyr::MvtSerializer::generateTile(bbox, z, pimpl->reader);

    // if they want you do to do the cleanup automatically
    if (auto_cleanup) {
      cleanup();
    }

    return mvt_data;
  } else {
    // Return error if no tile coordinates provided
    throw valhalla_exception_t{107, "Tile request requires tile coordinates (z/x/y) or bounding box locations"};
  }
}

std::string
actor_t::tile_xyz(uint32_t z, uint32_t x, uint32_t y, const std::function<void()>* interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);

  // Convert tile coordinates (z/x/y) to bounding box
  // This follows the standard Web Mercator tile calculation
  double n = std::pow(2.0, z);
  double west = x / n * 360.0 - 180.0;
  double east = (x + 1) / n * 360.0 - 180.0;
  double north = std::atan(std::sinh(M_PI * (1 - 2 * y / n))) * 180.0 / M_PI;
  double south = std::atan(std::sinh(M_PI * (1 - 2 * (y + 1) / n))) * 180.0 / M_PI;

  // Create bounding box from tile coordinates
  midgard::PointLL sw(south, west);
  midgard::PointLL ne(north, east);
  auto bbox = midgard::AABB2<midgard::PointLL>(west, south, east, north);

  // Generate MVT tile
  auto mvt_data = tyr::MvtSerializer::generateTile(bbox, z, pimpl->reader);

  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }

  return mvt_data;
}

} // namespace tyr
} // namespace valhalla
