// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/log_payloads/MeasurementStatisticsLogPayload.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"
#include "navkit/io/log_products/GnssPositionLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionUpdateLogProduct.hpp"
#include "navkit/io/log_products/NavEstimateLogProduct.hpp"
#include "navkit/io/log_products/TruthLogProduct.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace navkit::io
{

class RunLogger
{
public:
    RunLogger(std::filesystem::path output_dir, std::string run_name, nlohmann::json config)
        : m_output_dir(std::move(output_dir))
        , m_run_name(std::move(run_name))
        , m_config(std::move(config))
    {
        std::filesystem::create_directories(m_output_dir);

        m_truth_log.open(m_output_dir);
        m_gnss_log.open(m_output_dir);
        m_nav_log.open(m_output_dir);
        m_gnss_pos_update_log.open(m_output_dir);
    }

    void set_gnss_metadata(double sigma_h_m, double sigma_v_m, unsigned int seed)
    {
        m_gnss_log.set_metadata(sigma_h_m, sigma_v_m, seed);
    }

    void log_truth(const sim::TruthSample& sample)
    {
        m_truth_log.log(sample);
    }

    template<typename Measurement>
    void log_gnss(const Measurement& meas)
    {
        m_gnss_log.log(meas);
    }

    template<typename StateDef, typename Filter>
    void log_nav(double time_s, const Filter& filter, const sim::TruthSample& truth)
    {
        m_nav_log.log(NavEstimateLogPayload<StateDef, Filter>{
            .time_s = time_s,
            .filter = filter,
            .truth = truth,
        });
    }

    template<typename Statistics>
    void log_gnss_pos_statistics(const Statistics& stats)
    {
        m_gnss_pos_update_log.log(MeasurementStatisticsLogPayload<Statistics>{
            .statistics = stats,
        });
    }

    void close()
    {
        if (m_closed) {
            return;
        }

        m_truth_log.flush();
        m_gnss_log.flush();
        m_nav_log.flush();
        m_gnss_pos_update_log.flush();

        write_json_file(m_output_dir / "truth.meta.json", TruthLogProduct::metadata());
        write_json_file(m_output_dir / "gnss.meta.json", m_gnss_log.metadata());
        write_json_file(m_output_dir / "nav.meta.json", NavEstimateLogProduct::metadata());
        write_json_file(m_output_dir / "gnss_pos_update.meta.json",
                        GnssPositionUpdateLogProduct::metadata());
        write_json_file(m_output_dir / "run_manifest.json", run_manifest());

        m_closed = true;
    }

    const std::filesystem::path& output_dir() const
    {
        return m_output_dir;
    }

private:
    static void write_json_file(const std::filesystem::path& path, const nlohmann::json& j)
    {
        std::ofstream f(path);
        if (!f) {
            throw std::runtime_error("Failed to open JSON file: " + path.string());
        }
        f << std::setw(2) << j << '\n';
    }

    nlohmann::json run_manifest() const
    {
        return {{"run_name", m_run_name},
                {"config", m_config},
                {"logs",
                 {{"truth", TruthLogProduct::manifest_entry()},
                  {"gnss", GnssPositionLogProduct::manifest_entry()},
                  {"nav", NavEstimateLogProduct::manifest_entry()},
                  {"gnss_pos_update", GnssPositionUpdateLogProduct::manifest_entry()}}}};
    }

    std::filesystem::path m_output_dir;
    std::string m_run_name;
    nlohmann::json m_config;

    TruthLogProduct m_truth_log;
    GnssPositionLogProduct m_gnss_log;
    NavEstimateLogProduct m_nav_log;
    GnssPositionUpdateLogProduct m_gnss_pos_update_log;

    bool m_closed = false;
};

} // namespace navkit::io
