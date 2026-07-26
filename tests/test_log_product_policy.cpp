// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
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
#include "navkit/io/log_products/ImuDebugLogProduct.hpp"
#include "navkit/io/log_products/ImuIncrementLogProduct.hpp"
#include "navkit/io/log_products/NavEstimateLogProduct.hpp"
#include "navkit/io/log_products/TruthLogProduct.hpp"
#include "navkit/sim/ImuSimulator.hpp"
#include "navkit/sim/TruthSample.hpp"
#include "test_main.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <tuple>

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
using FilterCorrectionProduct = FilterCorrectionLogProduct<StateDef, Filter>;
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
    static_assert(
        LogProductPolicy<FilterCorrectionProduct, FilterCorrectionLogPayload<StateDef, Filter>>);
    static_assert(
        LogProductPolicy<GnssUpdateLogProduct, MeasurementStatisticsLogPayload<Statistics>>);

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
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<
                      FilterCorrectionLogPayload<StateDef, Filter>> == 1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<
                      MeasurementStatisticsLogPayload<Statistics>> == 1U);
    static_assert(EcefInsGnssTestRunLogger::matching_product_count_v<nlohmann::json> == 0U);

    static_assert(LoggerPolicy<EcefInsGnssTestRunLogger>);
    static_assert(LoggerPayloadPolicy<EcefInsGnssTestRunLogger, GnssPositionPayload>);
    static_assert(LoggerPayloadPolicy<EcefInsGnssTestRunLogger, ImuIncrementLogPayload>);
    static_assert(LoggerPayloadPolicy<EcefInsGnssTestRunLogger, ImuDebugLogPayload>);
    static_assert(LoggerPayloadPolicy<EcefInsGnssTestRunLogger,
                                      FilterCorrectionLogPayload<StateDef, Filter>>);
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

} // namespace navkit::io::test
