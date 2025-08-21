#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/worker.h"

// OR-Tools includes for VRP functionality
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_enums.pb.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;
using namespace operations_research;

namespace valhalla {
namespace thor {

// Basic OR-Tools test function to verify compilation
bool test_ortools_compilation() {
  LOG_INFO("Testing OR-Tools compilation...");

  // Create a simple routing model to test
  const int num_locations = 4;
  const int num_vehicles = 2;
  const RoutingIndexManager::NodeIndex depot{0};

  RoutingIndexManager manager(num_locations, num_vehicles, depot);
  RoutingModel routing(manager);

  LOG_INFO("OR-Tools compilation test successful!");
  return true;
}

void thor_worker_t::vehicle_routing(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // Test OR-Tools compilation
  if (!test_ortools_compilation()) {
    throw valhalla_exception_t{500, "OR-Tools compilation test failed"};
  }

  auto& options = *request.mutable_options();
  adjust_scores(options);
  auto costing = parse_costing(request);
  controller = AttributesController(options);

  // Use CostMatrix to find costs from each location to every other location
  costmatrix_.set_has_time(check_matrix_time(request, Matrix::CostMatrix));
  costmatrix_.SourceToTarget(request, *reader, mode_costing, mode,
                             max_matrix_distance.find(costing)->second);

  // Get the locations (depot + delivery points)
  const auto& locations = options.locations();
  const int num_locations = locations.size();

  if (num_locations < 2) {
    throw valhalla_exception_t{400, "Vehicle routing requires at least 2 locations (depot + delivery)"};
  }

  // Get the cost matrix from the request
  const auto& times = request.matrix().times();
  const auto& distances = request.matrix().distances();

  if (times.size() != num_locations * num_locations) {
    throw valhalla_exception_t{400, "Invalid cost matrix size"};
  }

  // Convert to OR-Tools format
  std::vector<std::vector<int64_t>> cost_matrix(num_locations, std::vector<int64_t>(num_locations));
  for (int i = 0; i < num_locations; ++i) {
    for (int j = 0; j < num_locations; ++j) {
      int64_t cost = static_cast<int64_t>(times.Get(i * num_locations + j));
      if (cost == kMaxCost) {
        cost = 1000000; // Large penalty for unreachable locations
      }
      cost_matrix[i][j] = cost;
    }
  }

  // Create OR-Tools routing model
  const RoutingIndexManager::NodeIndex depot{0};
  const int num_vehicles = 2; // Default to 2 vehicles, could be configurable

  RoutingIndexManager manager(num_locations, num_vehicles, depot);
  RoutingModel routing(manager);

  // Create the distance callback
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&cost_matrix, &manager](int64_t from_index, int64_t to_index) -> int64_t {
        auto from_node = manager.IndexToNode(from_index).value();
        auto to_node = manager.IndexToNode(to_index).value();
        return cost_matrix[from_node][to_node];
      });

  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Add distance constraint
  routing.AddDimension(transit_callback_index, 0, 30000, true, "Distance");
  routing.GetMutableDimension("Distance")->SetGlobalSpanCostCoefficient(100);

  // Setting first solution heuristic
  RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  // Solve the problem
  const Assignment* solution = routing.SolveWithParameters(search_parameters);

  if (!solution) {
    throw valhalla_exception_t{500, "No solution found for vehicle routing problem"};
  }

  LOG_INFO("Vehicle routing solution found!");

  // Extract the routes
  std::vector<std::vector<int>> routes;
  for (int vehicle_id = 0; vehicle_id < num_vehicles; ++vehicle_id) {
    std::vector<int> route;
    int64_t index = routing.Start(vehicle_id);

    while (!routing.IsEnd(index)) {
      route.push_back(manager.IndexToNode(index).value());
      index = solution->Value(routing.NextVar(index));
    }
    route.push_back(manager.IndexToNode(index).value());

    if (route.size() > 1) { // Only add non-empty routes
      routes.push_back(route);
    }
  }

  // Log the solution
  for (size_t i = 0; i < routes.size(); ++i) {
    std::string route_str = "Vehicle " + std::to_string(i) + ": ";
    for (int location : routes[i]) {
      route_str += std::to_string(location) + " -> ";
    }
    route_str = route_str.substr(0, route_str.length() - 4); // Remove last " -> "
    LOG_INFO(route_str);
  }

  // For now, just return the first route as a regular route
  // In a full implementation, you'd want to return multiple routes
  if (!routes.empty() && !routes[0].empty()) {
    // Reorder locations based on the first vehicle's route
    options.mutable_locations()->Clear();
    for (int location_index : routes[0]) {
      options.mutable_locations()->Add()->CopyFrom(locations.Get(location_index));
    }

    // Run the route
    path_depart_at(request, costing);
  } else {
    throw valhalla_exception_t{500, "No valid routes found"};
  }
}

} // namespace thor
} // namespace valhalla
