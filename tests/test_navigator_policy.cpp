// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/navigator/NavigatorUpdatePolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/navigator/propagation/ImuBiasDynamics.hpp"
#include "navkit/core/estimation/navigator/propagation/ImuProcessNoise.hpp"
#include "navkit/core/estimation/navigator/propagation/PropagationPolicies.hpp"
#include "navkit/core/estimation/navigator/propagation/PropagationPolicy.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicy.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "test_main.hpp"

#include <tuple>
#include <type_traits>

namespace navkit::core::estimation::test
{

using NavigatorPolicyStateDef = InsGyroAccelBiasStateDef;
using NavigatorPolicyModel = navkit::core::models::GnssPosModel<NavigatorPolicyStateDef>;
using NavigatorPolicySensor = Sensor<0U, NavigatorPolicyModel, 4>;
using NavigatorPolicyFilter = KalmanFilter<NavigatorPolicyStateDef>;
using NavigatorPolicySensors = std::tuple<NavigatorPolicySensor>;
using NavigatorPolicyPropagation = NoOpPropagation;
using NavigatorPolicyUpdate = UpdatePostFilter<NavigatorPolicyFilter>;

struct MissingFilterLifecycle
{};

struct MissingSensorProcessing
{
    using StateDef_t = NavigatorPolicyStateDef;
    using State_t = NavigatorPolicyFilter::State_t;
    using ErrorState_t = NavigatorPolicyFilter::ErrorState_t;
    using P_t = NavigatorPolicyFilter::P_t;

    [[nodiscard]] State_t& state();
    [[nodiscard]] const State_t& state() const;
    [[nodiscard]] P_t& covariance();
    [[nodiscard]] const P_t& covariance() const;
    void set_state(const State_t&);
    void set_covariance(const P_t&);
    void propagate_covariance(const P_t&, const P_t&);
    void set_covariance_floor(const P_t&);
    [[nodiscard]] const P_t& covariance_floor() const;
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

struct MissingStrapdownIntegration
{};

struct PropagationPolicyTestBase
{
    using ProcessNoise_t = ImuProcessNoise;
    using ImuBiasDynamics_t = GaussMarkovImuBiasDynamics;
    struct RuntimeConfig_t
    {
        ProcessNoise_t process_noise{};
        ImuBiasDynamics_t imu_bias_dynamics{};
    };

    inline static const RuntimeConfig_t runtime_config{};

    void set_runtime_config(const RuntimeConfig_t& config)
    {
        m_runtime_config = config;
    }

    [[nodiscard]] const RuntimeConfig_t& runtime_config_value() const
    {
        return m_runtime_config;
    }

private:
    RuntimeConfig_t m_runtime_config{};
};

struct WrongStrapdownIntegrationReturn : PropagationPolicyTestBase
{
    static constexpr std::size_t imu_buffer_capacity = 2U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr std::size_t covariance_history_capacity =
        2U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr Time_t covariance_update_rate_hz =
        1.0; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr bool apply_coning_sculling_compensation =
        false; // NOLINT(readability-identifier-naming)

    template<typename StateDef>
    int process_imu_increment(const ImuIncrement& increment, NominalState<StateDef>& state)
    {
        static_cast<void>(state);
        static_cast<void>(increment);
        return 0;
    }

    template<typename StateDef>
    bool process_imu_increment_pair(const ImuIncrement& first,
                                    const ImuIncrement& second,
                                    NominalState<StateDef>& state)
    {
        static_cast<void>(state);
        static_cast<void>(first);
        static_cast<void>(second);
        return true;
    }

    template<typename StateDef>
    bool covariance_step_from_increment(const NominalState<StateDef>& state,
                                        const ImuIncrement& increment,
                                        ErrorStateCov<StateDef>& phi,
                                        ErrorStateCov<StateDef>& qd) const
    {
        static_cast<void>(state);
        static_cast<void>(increment);
        phi.setIdentity();
        qd.setZero();
        return true;
    }

    template<typename StateDef>
    bool covariance_step_from_increment_pair(const NominalState<StateDef>& state,
                                             const ImuIncrement& first,
                                             const ImuIncrement& second,
                                             ErrorStateCov<StateDef>& phi,
                                             ErrorStateCov<StateDef>& qd) const
    {
        static_cast<void>(state);
        static_cast<void>(first);
        static_cast<void>(second);
        phi.setIdentity();
        qd.setZero();
        return true;
    }
};

struct MissingCovariancePrediction : PropagationPolicyTestBase
{
    static constexpr std::size_t imu_buffer_capacity = 2U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr std::size_t covariance_history_capacity =
        2U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr Time_t covariance_update_rate_hz =
        1.0; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr bool apply_coning_sculling_compensation =
        false; // NOLINT(readability-identifier-naming)

    template<typename StateDef>
    bool process_imu_increment(const ImuIncrement& increment, NominalState<StateDef>& state)
    {
        static_cast<void>(state);
        static_cast<void>(increment);
        return true;
    }

    template<typename StateDef>
    bool process_imu_increment_pair(const ImuIncrement& first,
                                    const ImuIncrement& second,
                                    NominalState<StateDef>& state)
    {
        static_cast<void>(state);
        static_cast<void>(first);
        static_cast<void>(second);
        return true;
    }
};

struct MissingImuProcessing : PropagationPolicyTestBase
{
    static constexpr std::size_t imu_buffer_capacity = 2U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr std::size_t covariance_history_capacity =
        2U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr Time_t covariance_update_rate_hz =
        1.0; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr bool apply_coning_sculling_compensation =
        false; // NOLINT(readability-identifier-naming)

    template<typename StateDef>
    bool covariance_step_from_increment(const NominalState<StateDef>& state,
                                        const ImuIncrement& increment,
                                        ErrorStateCov<StateDef>& phi,
                                        ErrorStateCov<StateDef>& qd) const
    {
        static_cast<void>(state);
        static_cast<void>(increment);
        phi.setIdentity();
        qd.setZero();
        return true;
    }

    template<typename StateDef>
    bool covariance_step_from_increment_pair(const NominalState<StateDef>& state,
                                             const ImuIncrement& first,
                                             const ImuIncrement& second,
                                             ErrorStateCov<StateDef>& phi,
                                             ErrorStateCov<StateDef>& qd) const
    {
        static_cast<void>(state);
        static_cast<void>(first);
        static_cast<void>(second);
        phi.setIdentity();
        qd.setZero();
        return true;
    }
};

struct RecordingPropagation : PropagationPolicyTestBase
{
    static constexpr std::size_t imu_buffer_capacity = 4U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr std::size_t covariance_history_capacity =
        4U; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr Time_t covariance_update_rate_hz =
        1.0; // NOLINT(readability-identifier-naming)
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr bool apply_coning_sculling_compensation =
        false; // NOLINT(readability-identifier-naming)
    static inline int imu_increment_call_count{0};
    static inline int imu_pair_call_count{0};
    static inline int covariance_increment_call_count{0};
    static inline int covariance_pair_call_count{0};
    static inline double last_imu_time_s{0.0};
    static inline RuntimeConfig_t last_runtime_config{};

    static void reset()
    {
        imu_increment_call_count = 0;
        imu_pair_call_count = 0;
        covariance_increment_call_count = 0;
        covariance_pair_call_count = 0;
        last_imu_time_s = 0.0;
        last_runtime_config = {};
    }

    template<typename StateDef>
    bool process_imu_increment(const ImuIncrement& increment, NominalState<StateDef>& state)
    {
        static_cast<void>(state);
        ++imu_increment_call_count;
        last_imu_time_s = increment.time_s;
        return increment.dt_s > 0.0;
    }

    template<typename StateDef>
    bool process_imu_increment_pair(const ImuIncrement& first,
                                    const ImuIncrement& second,
                                    NominalState<StateDef>& state)
    {
        static_cast<void>(state);
        ++imu_pair_call_count;
        last_imu_time_s = second.time_s;
        return first.dt_s > 0.0 && second.dt_s > 0.0;
    }

    template<typename StateDef>
    bool covariance_step_from_increment(const NominalState<StateDef>& state,
                                        const ImuIncrement& increment,
                                        ErrorStateCov<StateDef>& phi,
                                        ErrorStateCov<StateDef>& qd) const
    {
        static_cast<void>(state);
        ++covariance_increment_call_count;
        last_runtime_config = runtime_config_value();
        phi.setIdentity();
        qd.setZero();
        return increment.dt_s > 0.0;
    }

    template<typename StateDef>
    bool covariance_step_from_increment_pair(const NominalState<StateDef>& state,
                                             const ImuIncrement& first,
                                             const ImuIncrement& second,
                                             ErrorStateCov<StateDef>& phi,
                                             ErrorStateCov<StateDef>& qd) const
    {
        static_cast<void>(state);
        ++covariance_pair_call_count;
        last_runtime_config = runtime_config_value();
        phi.setIdentity();
        qd.setZero();
        return first.dt_s > 0.0 && second.dt_s > 0.0;
    }
};

template<typename Filter, typename Sensors>
concept CanInstantiateNavigator = requires { typename Navigator<Filter, Sensors>; };

template<typename Propagation>
concept CanInstantiateNavigatorWithPropagation =
    requires { typename Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, Propagation>; };

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

TEST_CASE("PropagationPolicy captures state propagation and covariance-step construction")
{
    static_assert(PropagationPolicy<NavigatorPolicyPropagation, NavigatorPolicyStateDef>);
    static_assert(PropagationPolicy<RecordingPropagation, NavigatorPolicyStateDef>);
    static_assert(!PropagationPolicy<MissingStrapdownIntegration, NavigatorPolicyStateDef>);
    static_assert(!PropagationPolicy<MissingCovariancePrediction, NavigatorPolicyStateDef>);
    static_assert(!PropagationPolicy<MissingImuProcessing, NavigatorPolicyStateDef>);
    static_assert(!PropagationPolicy<WrongStrapdownIntegrationReturn, NavigatorPolicyStateDef>);

    CHECK(true);
}

TEST_CASE(
    "Navigator is constrained by filter, sensor collection, propagation, and update boundaries")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors>;

    static_assert(std::is_default_constructible_v<Nav>);
    static_assert(CanInstantiateNavigator<NavigatorPolicyFilter, NavigatorPolicySensors>);
    static_assert(!CanInstantiateNavigator<MissingFilterLifecycle, NavigatorPolicySensors>);
    static_assert(!CanInstantiateNavigator<MissingSensorProcessing, NavigatorPolicySensors>);
    static_assert(!CanInstantiateNavigator<NavigatorPolicyFilter, NavigatorPolicySensor>);
    static_assert(CanInstantiateNavigatorWithPropagation<NavigatorPolicyPropagation>);
    static_assert(!CanInstantiateNavigatorWithPropagation<MissingStrapdownIntegration>);
    static_assert(!CanInstantiateNavigatorWithPropagation<WrongStrapdownIntegrationReturn>);

    CHECK(true);
}

TEST_CASE("Navigator covariance stage drains pending covariance steps")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;

    RecordingPropagation::reset();
    Nav navigator;

    CHECK(navigator.push_imu(ImuIncrement{.time_s = 1.0, .dt_s = 1.0}));
    CHECK(navigator.process_strapdown_integration());
    CHECK(navigator.pending_covariance_step_count() == 1U);
    CHECK(navigator.propagate_covariance());

    CHECK(navigator.pending_covariance_step_count() == 0U);
    CHECK(navigator.covariance_history_size() == 1U);
    CHECK(RecordingPropagation::covariance_increment_call_count == 1);
}

TEST_CASE("Navigator propagation owns selected runtime covariance construction config")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;

    RecordingPropagation::reset();
    Nav navigator;
    RecordingPropagation::RuntimeConfig_t runtime_config{};
    runtime_config.imu_bias_dynamics.gyro_bias_correlation_rate_1ps =
        navkit::core::Vec3::Constant(0.25);
    navigator.propagation().set_runtime_config(runtime_config);

    CHECK(navigator.push_imu(ImuIncrement{.time_s = 1.0, .dt_s = 1.0}));
    CHECK(navigator.process_strapdown_integration());

    CHECK(RecordingPropagation::covariance_increment_call_count == 1);
    CHECK(RecordingPropagation::last_runtime_config.imu_bias_dynamics.gyro_bias_correlation_rate_1ps
              .x() == doctest::Approx(0.25));
}

TEST_CASE("KalmanFilter applies selected covariance floor")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;
    using P = NavigatorPolicyFilter::P_t;

    Nav navigator;
    navigator.filter().set_covariance(P::Zero());

    P floor = P::Zero();
    floor(0, 0) = 2.0;
    floor(1, 1) = 0.0;
    navigator.filter().set_covariance_floor(floor);

    CHECK(navigator.filter().covariance_floor()(0, 0) == doctest::Approx(2.0));
    CHECK(navigator.filter().covariance()(0, 0) == doctest::Approx(2.0));
    CHECK(navigator.filter().covariance()(1, 1) == doctest::Approx(0.0));
}

TEST_CASE("Navigator defers covariance propagation until the configured medium-rate interval")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;

    RecordingPropagation::reset();
    Nav navigator;

    CHECK(navigator.push_imu(ImuIncrement{.time_s = 0.25, .dt_s = 0.25}));
    CHECK(navigator.update());

    CHECK(navigator.pending_covariance_step_count() == 1U);
    CHECK(navigator.pending_covariance_dt_s() == doctest::Approx(0.25));
    CHECK(navigator.covariance_history_size() == 0U);

    CHECK(navigator.push_imu(ImuIncrement{.time_s = 1.0, .dt_s = 0.75}));
    CHECK(navigator.update());

    CHECK(navigator.pending_covariance_step_count() == 0U);
    CHECK(navigator.pending_covariance_dt_s() == doctest::Approx(0.0));
    CHECK(navigator.covariance_history_size() == 1U);
}

TEST_CASE("Navigator consumes queued IMU increments through propagation policy")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;

    RecordingPropagation::reset();
    Nav navigator;

    CHECK(navigator.push_imu(ImuIncrement{.time_s = 1.0, .dt_s = 1.0}));
    CHECK(navigator.push_imu(ImuIncrement{.time_s = 2.0, .dt_s = 1.0}));
    CHECK(navigator.push_imu(ImuIncrement{.time_s = 3.0, .dt_s = 1.0}));
    CHECK(navigator.imu_buffer_size() == 3U);

    CHECK(navigator.process_strapdown_integration());

    CHECK(navigator.imu_buffer_size() == 0U);
    CHECK(navigator.pending_covariance_step_count() == 1U);
    CHECK(navigator.last_propagation_success());
    CHECK(RecordingPropagation::imu_pair_call_count == 1);
    CHECK(RecordingPropagation::imu_increment_call_count == 1);
    CHECK(RecordingPropagation::covariance_pair_call_count == 1);
    CHECK(RecordingPropagation::covariance_increment_call_count == 1);
    CHECK(RecordingPropagation::last_imu_time_s == doctest::Approx(3.0));
}

TEST_CASE("Navigator reports IMU propagation failure")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;

    RecordingPropagation::reset();
    Nav navigator;

    CHECK(navigator.push_imu(ImuIncrement{.time_s = 1.0, .dt_s = 0.0}));

    CHECK_FALSE(navigator.process_strapdown_integration());
    CHECK_FALSE(navigator.last_propagation_success());
    CHECK(navigator.imu_buffer_size() == 0U);
}

TEST_CASE("Navigator update orchestrates strapdown and covariance propagation before measurements")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;

    RecordingPropagation::reset();
    Nav navigator;

    CHECK(navigator.push_imu(ImuIncrement{.time_s = 1.0, .dt_s = 1.0}));
    navigator.update();

    CHECK(RecordingPropagation::imu_increment_call_count == 1);
    CHECK(RecordingPropagation::covariance_increment_call_count == 1);
    CHECK(navigator.pending_covariance_step_count() == 0U);
}

TEST_CASE("Navigator measurement processing does not invoke propagation stages")
{
    using Nav = Navigator<NavigatorPolicyFilter, NavigatorPolicySensors, RecordingPropagation>;

    RecordingPropagation::reset();
    Nav navigator;

    navigator.process_measurements();

    CHECK(RecordingPropagation::imu_increment_call_count == 0);
    CHECK(RecordingPropagation::imu_pair_call_count == 0);
    CHECK(RecordingPropagation::covariance_increment_call_count == 0);
    CHECK(RecordingPropagation::covariance_pair_call_count == 0);
}

} // namespace navkit::core::estimation::test
