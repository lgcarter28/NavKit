// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/navigator/NavigatorUpdatePolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicy.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "test_main.hpp"

#include <tuple>
#include <type_traits>

namespace navkit::core::estimation::test
{

using NavigatorPolicyStateDef = InsStateDef;
using NavigatorPolicyModel = navkit::core::models::GnssPosModel<NavigatorPolicyStateDef>;
using NavigatorPolicySensor = Sensor<0U, NavigatorPolicyModel, 4>;
using NavigatorPolicyFilter = KalmanFilter<NavigatorPolicyStateDef>;
using NavigatorPolicySensors = std::tuple<NavigatorPolicySensor>;
using NavigatorPolicyUpdate = UpdatePostFilter<NavigatorPolicyFilter>;

struct MissingFilterLifecycle
{};

struct MissingSensorProcessing
{
    using State_t = NavigatorPolicyFilter::State_t;
    using P_t = NavigatorPolicyFilter::P_t;

    [[nodiscard]] State_t& state();
    [[nodiscard]] const State_t& state() const;
    [[nodiscard]] P_t& covariance();
    [[nodiscard]] const P_t& covariance() const;
    void set_state(const State_t&);
    void set_covariance(const P_t&);
    void inject();
    void reset();
};

struct MissingSensorUpdate
{
    static void filter_update(NavigatorPolicyFilter& unused_filter)
    {
        static_cast<void>(unused_filter);
    }
};

struct MissingFilterUpdate
{
    template<typename Sensor>
    static void sensor_update(NavigatorPolicyFilter& unused_filter, const Sensor& unused_sensor)
    {
        static_cast<void>(unused_filter);
        static_cast<void>(unused_sensor);
    }
};

template<typename Filter, typename Sensors>
concept CanInstantiateNavigator = requires { typename Navigator<Filter, Sensors>; };

TEST_CASE("FilterPolicy captures the standalone filter lifecycle")
{
    static_assert(FilterCorrectionPolicy<NavigatorPolicyFilter>);
    static_assert(FilterCorrectionPolicy<MissingSensorProcessing>);
    static_assert(!FilterCorrectionPolicy<MissingFilterLifecycle>);

    static_assert(FilterPolicy<NavigatorPolicyFilter>);
    static_assert(FilterPolicy<MissingSensorProcessing>);
    static_assert(!FilterPolicy<MissingFilterLifecycle>);

    CHECK(true);
}

TEST_CASE("FilterSensorPolicy captures sensor-dependent filter processing")
{
    static_assert(FilterSensorPolicy<NavigatorPolicyFilter, NavigatorPolicySensor>);
    static_assert(!FilterSensorPolicy<MissingSensorProcessing, NavigatorPolicySensor>);

    CHECK(true);
}

TEST_CASE("SensorCollectionPolicy accepts tuple-like sensor collections")
{
    static_assert(SensorCollectionPolicy<NavigatorPolicySensors>);
    static_assert(SensorCollectionPolicy<const NavigatorPolicySensors&>);
    static_assert(!SensorCollectionPolicy<NavigatorPolicySensor>);
    static_assert(!SensorCollectionPolicy<std::tuple<int, double>>);

    CHECK(true);
}

TEST_CASE("UpdatePolicy captures Navigator update hooks")
{
    static_assert(
        UpdatePolicy<NavigatorPolicyUpdate, NavigatorPolicyFilter, NavigatorPolicySensor>);
    static_assert(UpdatePolicy<UpdateAfterEachSensor<NavigatorPolicyFilter>,
                               NavigatorPolicyFilter,
                               NavigatorPolicySensor>);
    static_assert(!UpdatePolicy<MissingSensorUpdate, NavigatorPolicyFilter, NavigatorPolicySensor>);
    static_assert(!UpdatePolicy<MissingFilterUpdate, NavigatorPolicyFilter, NavigatorPolicySensor>);

    CHECK(true);
}

TEST_CASE("NavigatorUpdatePolicy captures tuple-wide Navigator update compatibility")
{
    static_assert(NavigatorUpdatePolicy<NavigatorPolicyUpdate,
                                        NavigatorPolicyFilter,
                                        NavigatorPolicySensors>);
    static_assert(
        !NavigatorUpdatePolicy<MissingSensorUpdate, NavigatorPolicyFilter, NavigatorPolicySensors>);
    static_assert(
        !NavigatorUpdatePolicy<MissingFilterUpdate, NavigatorPolicyFilter, NavigatorPolicySensors>);

    CHECK(true);
}

TEST_CASE("Navigator is constrained by filter, sensor collection, and update boundaries")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors>;

    static_assert(std::is_default_constructible_v<Nav>);
    static_assert(CanInstantiateNavigator<NavigatorPolicyFilter, NavigatorPolicySensors>);
    static_assert(!CanInstantiateNavigator<MissingFilterLifecycle, NavigatorPolicySensors>);
    static_assert(!CanInstantiateNavigator<MissingSensorProcessing, NavigatorPolicySensors>);
    static_assert(!CanInstantiateNavigator<NavigatorPolicyFilter, NavigatorPolicySensor>);

    CHECK(true);
}

} // namespace navkit::core::estimation::test
