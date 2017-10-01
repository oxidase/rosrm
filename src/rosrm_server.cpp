#include <ros/ros.h>
#include <rosrm/RouteService.h>
#include <rosrm/MatchService.h>

#include <osrm/osrm.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/storage_config.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/json_container.hpp>

#include "json_renderer.hpp"
#include <sstream>

namespace
{
auto convert_coordinate(osrm::json::Array &coordinate)
{
    geometry_msgs::Point result;
    result.x = coordinate.values.at(0).get<osrm::util::json::Number>().value;
    result.y = coordinate.values.at(1).get<osrm::util::json::Number>().value;
    result.z = 0.;

    return result;
}

auto convert_coordinates(osrm::json::Array &coordinates)
{
    std::vector<geometry_msgs::Point> result;
    for (auto &coordinate : coordinates.values)
    {
        result.emplace_back(convert_coordinate(coordinate.get<osrm::util::json::Array>()));
    }

    return result;
}

auto convert_route(osrm::json::Object &route)
{
    rosrm::Route result;
    result.weight_name = route.values["weight_name"].get<osrm::util::json::String>().value;
    result.distance = route.values["distance"].get<osrm::util::json::Number>().value;
    result.duration = route.values["duration"].get<osrm::util::json::Number>().value;
    result.weight = route.values["weight"].get<osrm::util::json::Number>().value;

    auto &geometry = route.values["geometry"].get<osrm::util::json::Object>().values;
    result.coordinates = convert_coordinates(geometry["coordinates"].get<osrm::util::json::Array>());

    return result;
}

auto convert_matching(osrm::json::Object &matching)
{
    rosrm::Matching result;
    result.confidence = matching.values["confidence"].get<osrm::util::json::Number>().value;

    auto &geometry = matching.values["geometry"].get<osrm::util::json::Object>().values;
    result.coordinates = convert_coordinates(geometry["coordinates"].get<osrm::util::json::Array>());

    return result;
}
}

struct OSRMProxy
{
    OSRMProxy(const osrm::OSRM& osrm) : osrm(osrm) {}

    bool route(rosrm::RouteService::Request  &req, rosrm::RouteService::Response &res)
    {
        // Set route parameters
        osrm::RouteParameters parameters;

        for (const auto &waypoint : req.waypoints)
        {
            auto longitude = osrm::util::FloatLongitude{waypoint.position.x};
            auto latitude = osrm::util::FloatLatitude{waypoint.position.y};
            parameters.coordinates.emplace_back(longitude, latitude);
        }

        // Set fixed parameters
        parameters.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

        // TODO: add parameters

        // std::vector<boost::optional<Hint>> hints;
        // std::vector<boost::optional<double>> radiuses;
        // std::vector<boost::optional<Bearing>> bearings;
        // std::vector<boost::optional<Approach>> approaches;
        // std::vector<std::string> exclude;

        // bool steps = false;
        // unsigned number_of_alternatives = 0;
        // bool annotations = false;
        // AnnotationsType annotations_type = AnnotationsType::None;
        // OverviewType overview = OverviewType::Simplified;
        // boost::optional<bool> continue_straight;

        // Find a route
        osrm::json::Object response;
        const auto status = osrm.Route(parameters, response);

        std::stringstream sstr;
        osrm::util::json::render(sstr, response);
        ROS_INFO("%s", sstr.str().c_str());

        // Convert JSON response into a ROS message
        res.code = response.values["code"].get<osrm::util::json::String>().value;
        for (auto &route : response.values["routes"].get<osrm::json::Array>().values)
        {
            res.routes.emplace_back(convert_route(route.get<osrm::json::Object>()));
        }

        return status == osrm::engine::Status::Ok;
    }

    bool match(rosrm::MatchService::Request  &req, rosrm::MatchService::Response &res)
    {
        // Set map matching parameters
        osrm::MatchParameters parameters;
        std::vector<unsigned> timestamps;

        for (const auto &waypoint : req.waypoints)
        {
            auto longitude = osrm::util::FloatLongitude{waypoint.pose.position.x};
            auto latitude = osrm::util::FloatLatitude{waypoint.pose.position.y};
            parameters.coordinates.emplace_back(longitude, latitude);

            // TODO: add bearings from waypoint.pose.orientation

            timestamps.push_back(waypoint.header.stamp.toNSec() / 1000000000ull);
        }

        if (std::any_of(timestamps.begin(), timestamps.end(), [](auto x) { return x != 0; } ))
            parameters.timestamps = std::move(timestamps);

        // Set fixed parameters
        parameters.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

        // TODO: add parameters

        // std::vector<boost::optional<Hint>> hints;
        // std::vector<boost::optional<double>> radiuses;
        // std::vector<boost::optional<Bearing>> bearings;
        // std::vector<boost::optional<Approach>> approaches;
        // std::vector<std::string> exclude;

        // bool steps = false;
        // unsigned number_of_alternatives = 0;
        // bool annotations = false;
        // AnnotationsType annotations_type = AnnotationsType::None;
        // OverviewType overview = OverviewType::Simplified;
        // boost::optional<bool> continue_straight;

        // GapsType gaps;
        // bool tidy;

        // Do a map-matching
        osrm::json::Object response;
        const auto status = osrm.Match(parameters, response);

        std::stringstream sstr;
        osrm::util::json::render(sstr, response);
        ROS_INFO("%s", sstr.str().c_str());

        // // Convert JSON response into a ROS message
        res.code = response.values["code"].get<osrm::util::json::String>().value;
        for (auto &matching : response.values["matchings"].get<osrm::json::Array>().values)
        {
            res.matchings.emplace_back(convert_matching(matching.get<osrm::json::Object>()));
        }

        return status == osrm::engine::Status::Ok;
    }

    const osrm::OSRM &osrm;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosrm_server");
    ros::NodeHandle n;

    // Get parameters and fill config structures
    osrm::EngineConfig config;

    std::string base_path;
    n.param("base_path", base_path, std::string());
    config.storage_config = osrm::StorageConfig(base_path);

    std::string algorithm;
    n.param<std::string>("algorithm", algorithm, "CH");
    std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(), ::tolower);
    config.algorithm = algorithm == "mld" ? osrm::engine::EngineConfig::Algorithm::MLD
        : algorithm == "corech" ? osrm::engine::EngineConfig::Algorithm::CoreCH
        : osrm::engine::EngineConfig::Algorithm::CH;

    n.param("use_shared_memory", config.use_shared_memory, base_path.empty());


    // Create OSRM engine and ROS<->OSRM proxy
    osrm::OSRM osrm(config);
    OSRMProxy proxy(osrm);

    // Advertise OSRM service
    ros::ServiceServer service = n.advertiseService("rosrm/route", &OSRMProxy::route, &proxy);
    ros::ServiceServer service1 = n.advertiseService("rosrm/match", &OSRMProxy::match, &proxy);

    // Start service loop
    ROS_INFO("ROSRM is ready");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
