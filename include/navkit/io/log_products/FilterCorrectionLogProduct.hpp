// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/StateErrorLogSchema.hpp"
#include "navkit/io/log_payloads/FilterCorrectionLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

template<typename StateDef, typename Filter>
class FilterCorrectionLogProduct
{
public:
    static constexpr std::string_view LogKey = "filter_correction_ecef";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "filter_correction_ecef.csv",
                   detail::state_error_value_header<Error>("correction_"));
    }

    void log(const FilterCorrectionLogPayload<StateDef, Filter>& payload)
    {
        m_csv.write_row_values(
            detail::state_error_value_row<Error>(payload.time_s, payload.filter.last_correction()));
    }

    void flush()
    {
        m_csv.flush();
    }

    nlohmann::json metadata() const
    {
        return {{"schema", "filter_correction_ecef_v1"},
                {"file", "filter_correction_ecef.csv"},
                {"covariance", "none"},
                {"state_dimension", Error::N},
                {"description",
                 "Event log for actual filter correction vectors captured before injection/reset. "
                 "Covariance bounds are read from the nominal estimate/covariance log."}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "filter_correction_ecef.csv"},
                {"manifest", "filter_correction_ecef.meta.json"}};
    }

private:
    using Error = typename StateDef::Error;

    CsvWriter m_csv;
};

} // namespace navkit::io
