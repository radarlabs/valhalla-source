#pragma once

#include <vector>
#include <cstdint>

namespace valhalla {
namespace thor {

// Forward declaration to avoid including VROOM headers in public interface
struct VehicleRoutingSolution;

// Data structures for vehicle routing problem
struct Vehicle {
    uint32_t id;
    uint32_t start_location;
    uint32_t end_location;
};

struct Job {
    uint32_t id;
    uint32_t location;
    uint32_t service_time; // in seconds
};

struct RouteStep {
    enum class StepType {
        START = 0,
        JOB = 1,
        END = 2
    };
    
    StepType type;
    uint32_t location;
    uint32_t job_id;
};

struct Route {
    uint32_t vehicle_id;
    uint32_t cost;
    uint32_t duration;
    uint32_t service_time;
    std::vector<RouteStep> steps;
};

struct VehicleRoutingRequest {
    std::vector<Vehicle> vehicles;
    std::vector<Job> jobs;
    std::vector<std::vector<uint32_t>> distance_matrix;
};

struct VehicleRoutingSolution {
    uint32_t total_cost;
    uint32_t total_service_time;
    uint32_t total_duration;
    uint32_t computing_time_ms;
    std::vector<Route> routes;
    std::vector<uint32_t> unassigned_jobs;
};

/**
 * Solve a vehicle routing problem using VROOM
 * @param request The vehicle routing problem request
 * @return The solution containing optimal routes
 */
VehicleRoutingSolution solve_vehicle_routing_problem(const VehicleRoutingRequest& request);

/**
 * Example function that demonstrates basic VRP usage
 * Creates a simple problem with 1 vehicle and 3 jobs
 * @return The solution for the example problem
 */
VehicleRoutingSolution solve_basic_vrp_example();

} // namespace thor
} // namespace valhalla
