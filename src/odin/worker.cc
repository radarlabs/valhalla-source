#include "odin/worker.h"
#include "baldr/json.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "proto/trip.pb.h"
#include "tyr/serializers.h"
#include "tyr/actor.h"

#include <boost/property_tree/ptree.hpp>

#include <functional>
#include <string>
#include <sstream>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace odin {

odin_worker_t::odin_worker_t(const boost::property_tree::ptree& config)
    : service_worker_t(config), markup_formatter_(config), config_(config) {
  // signal that the worker started successfully
  started();
}

odin_worker_t::~odin_worker_t() {
}

std::string odin_worker_t::narrate(Api& request) const {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // get some annotated directions
  try {
    odin::DirectionsBuilder().Build(request, markup_formatter_);
  } catch (...) { throw valhalla_exception_t{202}; }

  // serialize those to the proper format
  return tyr::serializeDirections(request);
}

void odin_worker_t::status(Api&) const {
#ifdef ENABLE_SERVICES
  // if we are in the process of shutting down we signal that here
  // should react by draining traffic (though they are likely doing this as they are usually the ones
  // who sent us the request to shutdown)
  if (prime_server::draining() || prime_server::shutting_down()) {
    throw valhalla_exception_t{203};
  }
#endif
}

#ifdef ENABLE_SERVICES
prime_server::worker_t::result_t
odin_worker_t::work(const std::list<zmq::message_t>& job,
                    void* request_info,
                    const std::function<void()>& interrupt_function) {
  auto& info = *static_cast<prime_server::http_request_info_t*>(request_info);
  LOG_INFO("Got Odin Request " + std::to_string(info.id));
  LOG_INFO("ODIN DEBUG: Odin worker is being called");
  Api request;
  prime_server::worker_t::result_t result{false, {}, {}};
  try {
    // Set the interrupt function
    service_worker_t::set_interrupt(&interrupt_function);

    // crack open the in progress request
    bool success = request.ParseFromArray(job.front().data(), job.front().size());
    if (!success) {
      LOG_ERROR("Failed parsing pbf in Odin::Worker");
      throw valhalla_exception_t{200, "Failed parsing pbf in Odin::Worker"};
    }

    // its either a simple status request, tile request, or its a route to narrate
    switch (request.options().action()) {
      case Options::status: {
        status(request);
        auto response = tyr::serializeStatus(request);
        result = to_response(response, info, request);
        break;
      }
      case Options::tile: {
        LOG_INFO("ODIN DEBUG: Calling Tyr actor directly for tile request");
        // For tile requests, we need to call the Tyr actor directly
        // since the Tyr actor is not part of the worker pipeline

        // Extract tile coordinates and time parameter from HTTP request if not already in API options
        if (request.options().id().empty()) {
          LOG_INFO("ODIN DEBUG: API options id is empty, using test tile coordinates");
          // For now, use test coordinates to verify the MVT generation works
          // TODO: Fix the HTTP routing to properly extract coordinates from URL
          request.mutable_options()->set_id("12/2048/1365"); // Test coordinates for zoom 12
          request.mutable_options()->set_format(Options_Format_mvt);
          LOG_INFO("ODIN DEBUG: Set test tile coordinates: 12/2048/1365");
        }

        // Check if time parameter is set
        if (request.options().has_date_time_case()) {
          LOG_INFO("ODIN DEBUG: Time parameter set: " + request.options().date_time());
        } else {
          LOG_INFO("ODIN DEBUG: No time parameter set, using current traffic");
        }

        try {
          // Create a Tyr actor to handle the tile request
          valhalla::tyr::actor_t actor(config_);
          auto response = actor.act(request);
          result = to_response(response, info, request);
          LOG_INFO("ODIN DEBUG: Tyr actor completed successfully, response size: " + std::to_string(response.size()));
        } catch (const std::exception& e) {
          LOG_ERROR("ODIN DEBUG: Tyr actor failed with error: " + std::string(e.what()));
          throw valhalla_exception_t{400, std::string("Tyr actor failed: ") + e.what()};
        }
        break;
      }
      default: {
        // narrate them and serialize them along
        auto response = narrate(request);
        result = to_response(response, info, request);
        break;
      }
    }
  } catch (const std::exception& e) {
    result = serialize_error({299, std::string(e.what())}, info, request);
  }

  // keep track of the metrics if the request is going back to the client (this should be the case)
  if (!result.intermediate)
    enqueue_statistics(request);

  return result;
}

void run_service(const boost::property_tree::ptree& config) {
  // gracefully shutdown when asked via SIGTERM
  prime_server::quiesce(config.get<unsigned int>("httpd.service.drain_seconds", 28),
                        config.get<unsigned int>("httpd.service.shutting_seconds", 1));

  // gets requests from odin proxy
  auto upstream_endpoint = config.get<std::string>("odin.service.proxy") + "_out";
  // or returns just location information back to the server
  auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
  auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

  // listen for requests
  zmq::context_t context;
  odin_worker_t odin_worker(config);
  prime_server::worker_t worker(context, upstream_endpoint, "ipc:///dev/null", loopback_endpoint,
                                interrupt_endpoint,
                                std::bind(&odin_worker_t::work, std::ref(odin_worker),
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3),
                                std::bind(&odin_worker_t::cleanup, std::ref(odin_worker)));
  worker.work();

  // TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
}

#endif
} // namespace odin
} // namespace valhalla
