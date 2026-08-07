// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/models/GnssVelModel.hpp"
#include "navkit/io/LogProductPolicy.hpp"
#include "navkit/io/LoggerPolicy.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/io/log_payloads/FilterCorrectionLogPayload.hpp"
#include "navkit/io/log_payloads/GnssMeasurementLogPayload.hpp"
#include "navkit/io/log_payloads/ImuDebugLogPayload.hpp"
#include "navkit/io/log_payloads/ImuIncrementLogPayload.hpp"
#include "navkit/io/log_payloads/MeasurementStatisticsLogPayload.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"
#include "navkit/io/log_products/FilterCorrectionLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionUpdateLogProduct.hpp"
#include "navkit/io/log_products/GnssVelocityUpdateLogProduct.hpp"
#include "navkit/io/log_products/ImuDebugLogProduct.hpp"
#include "navkit/io/log_products/ImuIncrementLogProduct.hpp"
#include "navkit/io/log_products/NavEstimateLogProduct.hpp"
#include "navkit/io/log_products/TruthLogProduct.hpp"
#include "navkit/sim/sensors/ImuSimulator.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"
#include "test_main.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace navkit::io::test
{

namespace
{

using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
using Model = navkit::core::models::GnssPosModel<StateDef>;
using Sensor = navkit::core::estimation::Sensor<0U, Model, 4U>;
using Sensors = std::tuple<Sensor>;
using Filter = navkit::core::estimation::KalmanFilter<
    StateDef,
    navkit::core::estimation::DefaultInjectionPolicy<StateDef>,
    navkit::core::estimation::DefaultResetPolicy<StateDef>,
    Sensors>;
using GnssMeasurement = navkit::core::estimation::Measurement<3>;
using GnssPositionPayload = GnssPositionLogPayload;
using Statistics = navkit::core::estimation::MeasurementStatistics<Sensor>;
using GnssUpdateLogProduct = GnssPositionUpdateLogProduct<Statistics>;
using VelocityModel = navkit::core::models::GnssVelModel<StateDef>;
using VelocitySensor = navkit::core::estimation::Sensor<1U, VelocityModel, 4U>;
using VelocityStatistics = navkit::core::estimation::MeasurementStatistics<VelocitySensor>;
using GnssVelocityUpdateProduct = GnssVelocityUpdateLogProduct<VelocityStatistics>;
using FilterCorrectionProduct = FilterCorrectionLogProduct<StateDef>;
using EcefInsGnssTestRunLogger = RunLogger<TruthLogProduct,
                                           GnssPositionLogProduct,
                                           NavEstimateLogProduct<StateDef, Filter>,
                                           ImuIncrementLogProduct,
                                           ImuDebugLogProduct,
                                           FilterCorrectionProduct,
                                           GnssUpdateLogProduct>;

struct MissingOpen
{
    void log(const GnssMeasurement& /*measurement*/) {}
    void flush() {}
    static nlohmann::json metadata()
    {
        return {};
    }
    static nlohmann::json manifest_entry()
    {
        return {};
    }
};

struct MissingPayloadLog
{
    void open(const std::filesystem::path& /*output_dir*/) {}
    void log() {}
    void flush() {}
    static nlohmann::json metadata()
    {
        return {};
    }
    static nlohmann::json manifest_entry()
    {
        return {};
    }
};

struct MissingManifest
{
    void open(const std::filesystem::path& /*output_dir*/) {}
    void log(const GnssMeasurement& /*measurement*/) {}
    void flush() {}
    static nlohmann::json metadata()
    {
        return {};
    }
};

struct FakePayload
{
    int value{};
};

struct OtherFakePayload
{
    int value{};
};

class FakeLogProduct
{
public:
    static constexpr std::string_view LogKey = "fake";

    void open(const std::filesystem::path& output_dir)
    {
        m_path = output_dir / "fake.csv";
        std::ofstream file(m_path);
        file << "value\n";
    }

    void log(const FakePayload& payload)
    {
        ++m_log_count;
        std::ofstream file(m_path, std::ios::app);
        file << payload.value << '\n';
    }

    void flush()
    {
        ++m_flush_count;
    }

    [[nodiscard]] nlohmann::json metadata() const
    {
        return {{"schema", "fake_v1"}, {"log_count", m_log_count}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "fake.csv"}, {"manifest", "fake.meta.json"}};
    }

    [[nodiscard]] int flush_count() const
    {
        return m_flush_count;
    }

private:
    std::filesystem::path m_path;
    int m_log_count = 0;
    int m_flush_count = 0;
};

class OtherFakeLogProduct
{
public:
    static constexpr std::string_view LogKey = "other";

    void open(const std::filesystem::path& /*output_dir*/) {}
    void log(const OtherFakePayload& /*payload*/) {}
    void flush() {}
    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "other_v1"}};
    }
    static nlohmann::json manifest_entry()
    {
        return {{"csv", "other.csv"}, {"manifest", "other.meta.json"}};
    }
};

class AmbiguousFakeLogProduct
{
public:
    static constexpr std::string_view LogKey = "ambiguous";

    void open(const std::filesystem::path& /*output_dir*/) {}
    void log(const FakePayload& /*payload*/) {}
    void flush() {}
    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "ambiguous_v1"}};
    }
    static nlohmann::json manifest_entry()
    {
        return {{"csv", "ambiguous.csv"}, {"manifest", "ambiguous.meta.json"}};
    }
};

} // namespace

TEST_CASE("log product policies describe concrete payload boundaries")
{
    static_assert(LogProductPolicy<TruthLogProduct, navkit::sim::TruthSample>);
    static_assert(LogProductPolicy<GnssPositionLogProduct, GnssPositionPayload>);
    static_assert(LogProductPolicy<NavEstimateLogProduct<StateDef, Filter>,
                                   NavEstimateLogPayload<StateDef, Filter>>);
    static_assert(LogProductPolicy<ImuIncrementLogProduct, ImuIncrementLogPayload>);
    static_assert(LogProductPolicy<ImuDebugLogProduct, ImuDebugLogPayload>);
    static_assert(LogProductPolicy<FilterCorrectionProduct, FilterCorrectionLogPayload<StateDef>>);
    static_assert(
        LogProductPolicy<GnssUpdateLogProduct, MeasurementStatisticsLogPayload<Statistics>>);
    static_assert(LogProductPolicy<GnssVelocityUpdateProduct,
                                   MeasurementStatisticsLogPayload<VelocityStatistics>>);

    static_assert(!LogProductPolicy<MissingOpen, GnssMeasurement>);
    static_assert(!LogProductPolicy<MissingPayloadLog, GnssMeasurement>);
    static_assert(!LogProductPolicy<MissingManifest, GnssMeasurement>);

    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<navkit::sim::TruthSample> ==
                  1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<GnssPositionPayload> == 1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<
                      NavEstimateLogPayload<StateDef, Filter>> == 1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<ImuIncrementLogPayload> == 1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<ImuDebugLogPayload> == 1U);
    static_assert(
        EcefInsGnssTestRunLogger::matching_product_count_v<FilterCorrectionLogPayload<StateDef>> ==
        1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<
                      MeasurementStatisticsLogPayload<Statistics>> == 1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<nlohmann::json> == 0U);

    static_assert(LoggerPolicy<EcefInsGnssTestRunLogger>);
    static_assert(LoggerPayloadPolicy<EcefInsGnssTestRunLogger, GnssPositionPayload>);
    static_assert(LoggerPayloadPolicy<EcefInsGnssTestRunLogger, ImuIncrementLogPayload>);
    static_assert(LoggerPayloadPolicy<EcefInsGnssTestRunLogger, ImuDebugLogPayload>);
    static_assert(
        LoggerPayloadPolicy<EcefInsGnssTestRunLogger, FilterCorrectionLogPayload<StateDef>>);
    static_assert(LoggerProductAccessPolicy<EcefInsGnssTestRunLogger, GnssPositionLogProduct>);
    static_assert(LoggerProductAccessPolicy<EcefInsGnssTestRunLogger, GnssUpdateLogProduct>);

    CHECK(true);
}

TEST_CASE("RunLogger routes payloads to one selected product and writes manifests")
{
    const auto output_dir =
        std::filesystem::temp_directory_path() / "navkit_run_logger_test_output";
    std::filesystem::remove_all(output_dir);

    using Logger = RunLogger<FakeLogProduct, OtherFakeLogProduct>;
    static_assert(Logger::matching_product_count_v<FakePayload> == 1U);
    static_assert(Logger::matching_product_count_v<OtherFakePayload> == 1U);
    static_assert(Logger::matching_product_count_v<nlohmann::json> == 0U);
    static_assert(LoggerPolicy<Logger>);
    static_assert(LoggerPayloadPolicy<Logger, FakePayload>);
    static_assert(LoggerProductAccessPolicy<Logger, FakeLogProduct>);

    Logger logger(output_dir, "run_logger_test", nlohmann::json{{"config", "fake"}});
    logger.log(FakePayload{.value = 42});
    logger.close();

    CHECK(std::filesystem::exists(output_dir / "fake.csv"));
    CHECK(std::filesystem::exists(output_dir / "fake.meta.json"));
    CHECK(std::filesystem::exists(output_dir / "run_manifest.json"));

    {
        std::ifstream metadata_file(output_dir / "fake.meta.json");
        const auto metadata = nlohmann::json::parse(metadata_file);
        CHECK(metadata.at("schema") == "fake_v1");
        CHECK(metadata.at("log_count") == 1);
    }
    CHECK(logger.product<FakeLogProduct>().flush_count() == 1);

    {
        std::ifstream manifest_file(output_dir / "run_manifest.json");
        const auto manifest = nlohmann::json::parse(manifest_file);
        CHECK(manifest.at("run_name") == "run_logger_test");
        CHECK(manifest.at("logs").contains("fake"));
        CHECK(manifest.at("logs").contains("other"));
    }

    std::filesystem::remove_all(output_dir);
}

TEST_CASE("RunLogger exposes zero and ambiguous payload matches at compile time")
{
    using MissingPayloadLogger = RunLogger<FakeLogProduct>;
    using AmbiguousPayloadLogger = RunLogger<FakeLogProduct, AmbiguousFakeLogProduct>;

    static_assert(MissingPayloadLogger::matching_product_count_v<OtherFakePayload> == 0U);
    static_assert(!LoggerPayloadPolicy<MissingPayloadLogger, OtherFakePayload>);

    static_assert(AmbiguousPayloadLogger::matching_product_count_v<FakePayload> == 2U);
    static_assert(!LoggerPayloadPolicy<AmbiguousPayloadLogger, FakePayload>);

    CHECK(true);
}

TEST_CASE("filter-correction CSV exactly replays a composed same-cycle correction")
{
    using Nominal = StateDef::Nominal;
    using Error = StateDef::Error;

    Filter filter{};
    Filter::State_t initial_state = Filter::State_t::Zero();
    initial_state(Nominal::AttQuat::i) = 1.0;
    filter.set_state(initial_state);

    Filter::P_t covariance = Filter::P_t::Identity();
    covariance(Error::AttRotVec::i, Error::Pos::i) = 0.15;
    covariance(Error::Pos::i, Error::AttRotVec::i) = 0.15;
    covariance(Error::AttRotVec::i + 1, Error::Pos::i + 1) = -0.12;
    covariance(Error::Pos::i + 1, Error::AttRotVec::i + 1) = -0.12;
    filter.set_covariance(covariance);

    Model::ObservationContext context{};
    context.R_e_m2 = navkit::core::Mat3::Identity();
    Model::O_t first_measurement = Model::O_t::Zero();
    Model::O_t second_measurement = Model::O_t::Zero();
    first_measurement.x() = 2.0;
    second_measurement.y() = -3.0;

    filter.observation_update<Model>(first_measurement, context);
    const Filter::AppliedCorrection_t first_correction = filter.inject();
    filter.reset();
    filter.observation_update<Model>(second_measurement, context);
    const Filter::AppliedCorrection_t second_correction = filter.inject();
    filter.reset();
    const Filter::AppliedCorrection_t applied_correction =
        Filter::compose_applied_corrections(first_correction, second_correction);
    REQUIRE(applied_correction.valid);

    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "navkit_filter_correction_integrity_test";
    std::filesystem::remove_all(output_dir);
    std::filesystem::create_directories(output_dir);

    Filter::ErrorState_t logged_correction = Filter::ErrorState_t::Zero();
    {
        FilterCorrectionProduct product{};
        product.open(output_dir);
        product.log(FilterCorrectionLogPayload<StateDef>{
            .time_s = 5.0,
            .correction = applied_correction.value,
        });
        product.flush();

        std::ifstream csv{output_dir / "filter_correction_ecef.csv"};
        REQUIRE(csv.is_open());
        std::string header;
        std::string data_row;
        REQUIRE(static_cast<bool>(std::getline(csv, header)));
        REQUIRE(static_cast<bool>(std::getline(csv, data_row)));
        std::stringstream fields{data_row};
        std::vector<double> values;
        std::string field;
        while (std::getline(fields, field, ',')) {
            values.push_back(std::stod(field));
        }
        REQUIRE(values.size() == static_cast<std::size_t>(1 + Error::N));

        for (int index = 0; index < Error::N; ++index) {
            logged_correction(index) = values.at(static_cast<std::size_t>(index) + 1U);
        }
    }
    CHECK(logged_correction.isApprox(applied_correction.value, 1.0e-12));

    Filter::State_t replayed_state = initial_state;
    navkit::core::estimation::DefaultInjectionPolicy<StateDef>::apply(replayed_state,
                                                                      logged_correction);
    CHECK(replayed_state.isApprox(filter.state(), 1.0e-12));

    std::filesystem::remove_all(output_dir);
}

TEST_CASE("GNSS update products log versioned innovation-gate diagnostics")
{
    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "navkit_gnss_update_gate_log_test";
    std::filesystem::remove_all(output_dir);
    std::filesystem::create_directories(output_dir);

    Statistics position_stats{};
    position_stats.t.s = 12U;
    position_stats.accepted = false;
    position_stats.innovation_covariance_valid = true;
    position_stats.nis = 8.25;
    position_stats.gate_enabled = true;
    position_stats.gate_probability = 0.95;
    position_stats.gate_dof = 3U;
    position_stats.gate_threshold = 7.5;

    VelocityStatistics velocity_stats{};
    velocity_stats.t.s = 13U;
    velocity_stats.accepted = true;
    velocity_stats.innovation_covariance_valid = true;
    velocity_stats.nis = 2.5;
    velocity_stats.gate_enabled = true;
    velocity_stats.gate_probability = 0.99;
    velocity_stats.gate_dof = 3U;
    velocity_stats.gate_threshold = 11.0;

    {
        GnssUpdateLogProduct product{};
        product.open(output_dir);
        product.log(MeasurementStatisticsLogPayload<Statistics>{.statistics = position_stats});
        product.flush();
        CHECK(product.metadata().at("schema") == "gnss_pos_update_v2");
    }
    {
        GnssVelocityUpdateProduct product{};
        product.open(output_dir);
        product.log(
            MeasurementStatisticsLogPayload<VelocityStatistics>{.statistics = velocity_stats});
        product.flush();
        CHECK(product.metadata().at("schema") == "gnss_vel_update_v2");
    }

    for (const std::string filename : {"gnss_pos_update.csv", "gnss_vel_update.csv"}) {
        std::ifstream csv{output_dir / filename};
        REQUIRE(csv.is_open());
        std::string header;
        std::string row;
        REQUIRE(static_cast<bool>(std::getline(csv, header)));
        REQUIRE(static_cast<bool>(std::getline(csv, row)));
        CHECK(header.starts_with(
            "time_s,accepted,innovation_covariance_valid,nis,gate_enabled,gate_probability,"
            "gate_dof,gate_threshold"));

        std::stringstream header_stream{header};
        std::stringstream row_stream{row};
        std::vector<std::string> header_fields;
        std::vector<std::string> row_fields;
        std::string field;
        while (std::getline(header_stream, field, ',')) {
            header_fields.push_back(field);
        }
        while (std::getline(row_stream, field, ',')) {
            row_fields.push_back(field);
        }
        CHECK(row_fields.size() == header_fields.size());
        CHECK(std::stod(row_fields.at(6U)) == doctest::Approx(3.0));
    }

    std::filesystem::remove_all(output_dir);
}

} // namespace navkit::io::test
