#include <iostream>
#include <vector>
#include "valhalla/thor/vehicle_routing_problem.h"

#ifdef VROOM_FOUND
#include <vroom/vroom.h>
#endif

namespace valhalla {
namespace thor {

VehicleRoutingSolution solve_vehicle_routing_problem(const VehicleRoutingRequest& request) {
    VehicleRoutingSolution solution;


#ifdef VROOM_FOUND
    // Create a VROOM problem instance
    vroom::Input problem;

    // Add vehicles from the request
    for (const auto& vehicle : request.vehicles) {
        problem.add_vehicle(vroom::Vehicle(vehicle.id, vehicle.start_location, vehicle.end_location));
    }

    // Add jobs from the request
    for (const auto& job : request.jobs) {
        problem.add_job(vroom::Job(job.id, job.location, job.service_time));
    }

    // Set the distance matrix
    problem.set_matrix(request.distance_matrix);

    // Solve the problem
    vroom::Solution vroom_solution = problem.solve();

    // Convert VROOM solution to our format
    solution.total_cost = vroom_solution.summary.cost;
    solution.total_service_time = vroom_solution.summary.service;
    solution.total_duration = vroom_solution.summary.duration;
    solution.computing_time_ms = vroom_solution.summary.computing_times.solving;

    // Convert routes
    for (const auto& route : vroom_solution.routes) {
        Route route_info;
        route_info.vehicle_id = route.vehicle;
        route_info.cost = route.cost;
        route_info.duration = route.duration;
        route_info.service_time = route.service;

        // Convert steps
        for (const auto& step : route.steps) {
            RouteStep step_info;
            step_info.type = static_cast<RouteStep::StepType>(step.type);
            step_info.location = step.location;
            step_info.job_id = step.job;
            route_info.steps.push_back(step_info);
        }

        solution.routes.push_back(route_info);
    }

    // Convert unassigned jobs
    for (const auto& job : vroom_solution.unassigned) {
        solution.unassigned_jobs.push_back(job.id);
    }
#else
    // VROOM not available - return empty solution
    solution.total_cost = 0;
    solution.total_service_time = 0;
    solution.total_duration = 0;
    solution.computing_time_ms = 0;
#endif

    return solution;
}

// Example function that demonstrates basic VRP usage
VehicleRoutingSolution solve_basic_vrp_example() {
    VehicleRoutingRequest request;

    // Add one vehicle starting and ending at location 0
    Vehicle vehicle;
    vehicle.id = 1;
    vehicle.start_location = 0;
    vehicle.end_location = 0;
    request.vehicles.push_back(vehicle);

    // Add three jobs
    Job job1, job2, job3;
    job1.id = 1;
    job1.location = 1;
    job1.service_time = 300; // 5 minutes

    job2.id = 2;
    job2.location = 2;
    job2.service_time = 300;

    job3.id = 3;
    job3.location = 3;
    job3.service_time = 300;

    request.jobs = {job1, job2, job3};

    // Define distance matrix (4x4 symmetric matrix)
    // Locations: 0 (depot), 1, 2, 3
    request.distance_matrix = {
        {0, 10, 15, 20},  // from depot to locations
        {10, 0, 35, 25},  // from location 1
        {15, 35, 0, 30},  // from location 2
        {20, 25, 30, 0}   // from location 3
    };

    return solve_vehicle_routing_problem(request);
}

} // namespace thor
} // namespace valhalla
