#include <rosrm/RouteService.h>
#include <rosrm/MatchService.h>

#include <osrm/osrm.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/storage_config.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/json_container.hpp>
#include <util/json_renderer.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

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

auto convert_lane(osrm::json::Object &lane)
{
    rosrm::Lane result;

    result.valid = lane.values["valid"].which() == 4;
    auto &indications = lane.values["indications"].get<osrm::util::json::Array>().values;
    for (auto &indication : indications)
    {
        result.indications.emplace_back(indication.get<osrm::util::json::String>().value);
    }

    return result;
}

auto convert_intersection(osrm::json::Object &intersection)
{
    rosrm::Intersection result;

    result.in = intersection.values.count("in") ? intersection.values["in"].get<osrm::util::json::Number>().value : 0;
    result.out = intersection.values.count("out") ? intersection.values["out"].get<osrm::util::json::Number>().value : 0;
    result.location = convert_coordinate(intersection.values["location"].get<osrm::util::json::Array>());

    auto &bearings = intersection.values["bearings"].get<osrm::util::json::Array>().values;
    for (auto &bearing : bearings)
    {
        result.bearings.emplace_back(bearing.get<osrm::util::json::Number>().value);
    }

    auto &entries = intersection.values["entry"].get<osrm::util::json::Array>().values;
    for (auto &entry : entries)
    {
        result.entry.emplace_back(entry.which() == 4);
    }

    auto lanes = intersection.values.find("lanes");
    if (lanes != intersection.values.end())
    {
        for (auto &lane : lanes->second.get<osrm::util::json::Array>().values)
        {
            result.lanes.emplace_back(convert_lane(lane.get<osrm::util::json::Object>()));
        }
    }

    return result;
}

auto convert_maneuver(osrm::json::Object &maneuver)
{
    rosrm::Maneuver result;
    result.type = maneuver.values["type"].get<osrm::util::json::String>().value;
    result.modifier = maneuver.values["modifier"].get<osrm::util::json::String>().value;
    result.exit = maneuver.values.count("exit") ? maneuver.values["exit"].get<osrm::util::json::Number>().value : 0;
    result.bearing_before = maneuver.values["bearing_before"].get<osrm::util::json::Number>().value;
    result.bearing_after = maneuver.values["bearing_after"].get<osrm::util::json::Number>().value;
    result.location = convert_coordinate(maneuver.values["location"].get<osrm::util::json::Array>());

    return result;
}

auto convert_step(osrm::json::Object &step)
{
    rosrm::RouteStep result;
    result.name = step.values["name"].get<osrm::util::json::String>().value;
    result.mode = step.values["mode"].get<osrm::util::json::String>().value;
    result.distance = step.values["distance"].get<osrm::util::json::Number>().value;
    result.duration = step.values["duration"].get<osrm::util::json::Number>().value;
    result.weight = step.values["weight"].get<osrm::util::json::Number>().value;
    result.maneuver = convert_maneuver(step.values["maneuver"].get<osrm::util::json::Object>());

    auto &geometry = step.values["geometry"].get<osrm::util::json::Object>().values;
    result.coordinates = convert_coordinates(geometry["coordinates"].get<osrm::util::json::Array>());

    auto &intersections = step.values["intersections"].get<osrm::util::json::Array>().values;
    for (auto &intersection : intersections)
    {
        result.intersections.emplace_back(convert_intersection(intersection.get<osrm::util::json::Object>()));
    }

    return result;
}

auto convert_annotation(osrm::json::Object &annotation)
{
    rosrm::Annotation result;

    auto copy_values = [&annotation](const std::string &key, auto &vec) {
        auto &values = annotation.values;
        auto it = values.find(key);
        if (it != values.end())
        {
            for (auto &v : it->second.get<osrm::util::json::Array>().values)
            {
                vec.emplace_back(v.get<osrm::util::json::Number>().value);
            }
        }
    };

    copy_values("duration", result.duration);
    copy_values("nodes", result.nodes);
    copy_values("distance", result.distance);
    copy_values("weight", result.weight);
    copy_values("datasources", result.datasources);
    copy_values("speed", result.speed);

    return result;
}

auto convert_leg(osrm::json::Object &leg)
{
    rosrm::RouteLeg result;
    result.summary = leg.values["summary"].get<osrm::util::json::String>().value;
    result.distance = leg.values["distance"].get<osrm::util::json::Number>().value;
    result.duration = leg.values["duration"].get<osrm::util::json::Number>().value;
    result.weight = leg.values["weight"].get<osrm::util::json::Number>().value;

    auto &steps = leg.values["steps"].get<osrm::util::json::Array>().values;
    for (auto &step : steps)
    {
        result.steps.emplace_back(convert_step(step.get<osrm::util::json::Object>()));
    }

    if (leg.values.count("annotation"))
    {
        result.annotation = convert_annotation(leg.values["annotation"].get<osrm::util::json::Object>());
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

    auto &legs = route.values["legs"].get<osrm::util::json::Array>().values;
    for (auto &leg : legs)
    {
        result.legs.emplace_back(convert_leg(leg.get<osrm::util::json::Object>()));
    }

    return result;
}

auto convert_matching(osrm::json::Object &matching)
{
    rosrm::Matching result;
    result.confidence = matching.values["confidence"].get<osrm::util::json::Number>().value;

    auto &geometry = matching.values["geometry"].get<osrm::util::json::Object>().values;
    result.coordinates = convert_coordinates(geometry["coordinates"].get<osrm::util::json::Array>());

    auto &legs = matching.values["legs"].get<osrm::util::json::Array>().values;
    for (auto &leg : legs)
    {
        result.legs.emplace_back(convert_leg(leg.get<osrm::util::json::Object>()));
    }

    return result;
}

template<typename Request, typename Parameters>
void convert_base_parameters(const Request &request, Parameters &parameters)
{
    for (const auto &radius : request.radiuses)
    {
        parameters.radiuses.push_back(radius == 0. ? boost::optional<double>() : radius);
    }

    for (const auto &approach : request.approaches)
    {
        parameters.approaches.push_back(static_cast<osrm::engine::Approach>(approach));
    }

    parameters.exclude = request.exclude;
    parameters.steps = request.steps;
    parameters.number_of_alternatives = request.number_of_alternatives;
    parameters.annotations_type = static_cast<osrm::engine::api::RouteParameters::AnnotationsType>(request.annotation);
    parameters.overview = static_cast<osrm::engine::api::RouteParameters::OverviewType>(request.overview);
    parameters.continue_straight = request.continue_straight;
}

}

struct OSRMProxy
{
    OSRMProxy(const osrm::OSRM& osrm) : osrm(osrm) {}

    bool route(rosrm::RouteService::Request  &req, rosrm::RouteService::Response &res)
    {
        // Set route parameters
        osrm::RouteParameters parameters;

        // Set common parameters
        convert_base_parameters(req, parameters);

        // Set fixed parameters
        parameters.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

        // Set request waypoints and bearings
        for (const auto &waypoint : req.waypoints)
        {
            auto longitude = osrm::util::FloatLongitude{waypoint.position.x};
            auto latitude = osrm::util::FloatLatitude{waypoint.position.y};
            parameters.coordinates.emplace_back(longitude, latitude);
        }

        for (const auto &bearing : req.bearings)
        {
            parameters.bearings.push_back(osrm::engine::Bearing{bearing.bearing, bearing.range});
        }

        if (!parameters.IsValid())
        {
            res.code = "IncorrectParameters";
            return true;
        }

        // Find a route
        osrm::json::Object response;
        const auto status = osrm.Route(parameters, response);

        // std::stringstream sstr;
        // osrm::util::json::render(sstr, response);
        // ROS_INFO("%s", sstr.str().c_str());

        // Convert JSON response into a ROS message
        res.code = response.values["code"].get<osrm::util::json::String>().value;
        if (status != osrm::engine::Status::Ok)
            throw ros::Exception(response.values["message"].get<osrm::util::json::String>().value);

        for (auto &route : response.values["routes"].get<osrm::json::Array>().values)
        {
            res.routes.emplace_back(convert_route(route.get<osrm::json::Object>()));
        }

        return true;
    }

    bool match(rosrm::MatchService::Request  &req, rosrm::MatchService::Response &res)
    {
        // Set map matching parameters
        osrm::MatchParameters parameters;
        std::vector<unsigned> timestamps;

        // Set common parameters
        convert_base_parameters(req, parameters);

        // Set fixed parameters
        parameters.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

        // Set request waypoints, bearings and timestamps
        for (const auto &waypoint : req.waypoints)
        {
            auto longitude = osrm::util::FloatLongitude{waypoint.pose.position.x};
            auto latitude = osrm::util::FloatLatitude{waypoint.pose.position.y};
            parameters.coordinates.emplace_back(longitude, latitude);

            tf::Quaternion orientation(waypoint.pose.orientation.x, waypoint.pose.orientation.y,
                                       waypoint.pose.orientation.z, waypoint.pose.orientation.w);
            if (fabs(orientation.length2() - 1 ) < tf::QUATERNION_TOLERANCE / 2)
            {
                const auto yaw = tf::getYaw(orientation);
                const auto bearing = static_cast<short>(std::round((180. * yaw / M_PI) + (yaw < 0 ? 360. : 0.)));
                parameters.bearings.push_back(osrm::engine::Bearing{bearing, 10});
            }
            else
            {
                parameters.bearings.push_back(boost::none);
            }

            timestamps.push_back(waypoint.header.stamp.toNSec() / 1000000000ull);
        }

        if (std::any_of(timestamps.begin(), timestamps.end(), [](auto x) { return x != 0; } ))
            parameters.timestamps = std::move(timestamps);

        parameters.gaps = static_cast<osrm::engine::api::MatchParameters::GapsType>(req.gaps);
        parameters.tidy = req.tidy;

        if (!parameters.IsValid())
        {
            res.code = "IncorrectParameters";
            return true;
        }

        // Do a map-matching
        osrm::json::Object response;
        const auto status = osrm.Match(parameters, response);

        // std::stringstream sstr;
        // osrm::util::json::render(sstr, response);
        // ROS_INFO("%s", sstr.str().c_str());

        // Convert JSON response into a ROS message
        res.code = response.values["code"].get<osrm::util::json::String>().value;
        if (status != osrm::engine::Status::Ok)
            throw ros::Exception(response.values["message"].get<osrm::util::json::String>().value);

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
    ros::NodeHandle n("~");

    std::vector< std::string > keys;
    n.getParamNames(keys);
    for (auto x : keys)
        std::cout << x << "\n";

    // Get parameters and fill config structures
    osrm::EngineConfig config;

    std::string base_path;
    n.param("base_path", base_path, std::string());
    config.storage_config = osrm::StorageConfig(base_path);

    std::string algorithm;
    n.param<std::string>("algorithm", algorithm, "CH");
    std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(), ::tolower);
    config.algorithm =
        algorithm == "mld" ? osrm::engine::EngineConfig::Algorithm::MLD :
        algorithm == "corech" ? osrm::engine::EngineConfig::Algorithm::CoreCH :
        osrm::engine::EngineConfig::Algorithm::CH;

    n.param("use_shared_memory", config.use_shared_memory, base_path.empty());

    // Create OSRM engine and ROS<->OSRM proxy
    ROS_INFO("Starting ROSRM with %s using %s",
             config.use_shared_memory ? "shared memory" : base_path.c_str(),
             algorithm.c_str());

    try
    {
        osrm::OSRM osrm(config);
        OSRMProxy proxy(osrm);

        // Advertise OSRM service
        ros::ServiceServer service = n.advertiseService("route", &OSRMProxy::route, &proxy);
        ros::ServiceServer service1 = n.advertiseService("match", &OSRMProxy::match, &proxy);

        // Start service loop
        ROS_INFO("ROSRM is ready");
        ros::AsyncSpinner spinner(0);
        spinner.start();
        ros::waitForShutdown();
    }
    catch(const std::exception& exc)
    {
        ROS_ERROR(exc.what());
        return 1;
    }

    return 0;
}
