// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/io/CsvSchemaUtils.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace navkit::io::detail
{

enum class CovarianceLogMode
{
    Diagonal,
    Triangular
};

inline CovarianceLogMode covariance_log_mode_from_json(const nlohmann::json& cfg,
                                                       const char* logging_key)
{
    if (!cfg.contains("logging") || !cfg.at("logging").contains(logging_key)) {
        return CovarianceLogMode::Diagonal;
    }

    const nlohmann::json& logging = cfg.at("logging").at(logging_key);
    const std::string mode = logging.value("covariance", std::string("diagonal"));
    if (mode == "diagonal") {
        return CovarianceLogMode::Diagonal;
    }
    if (mode == "triangular") {
        return CovarianceLogMode::Triangular;
    }
    return CovarianceLogMode::Diagonal;
}

inline const char* covariance_log_mode_name(CovarianceLogMode mode)
{
    return mode == CovarianceLogMode::Triangular ? "triangular" : "diagonal";
}

template<typename Segment, typename Vector>
void append_segment_values(std::vector<double>& row, const Vector& values)
{
    const Eigen::Matrix<navkit::core::Scalar_t, Segment::sz, 1> segment_values =
        navkit::core::estimation::segment<Segment>(values);
    for (int axis = 0; axis < Segment::sz; ++axis) {
        row.push_back(segment_values(axis));
    }
}

template<typename Segment, typename Matrix>
void append_segment_sigma_values(std::vector<double>& row, const Matrix& covariance)
{
    for (int axis = 0; axis < Segment::sz; ++axis) {
        const int index = Segment::i + axis;
        row.push_back(std::sqrt(covariance(index, index)));
    }
}

template<typename Error>
std::vector<std::string> state_error_labels()
{
    std::vector<std::string> labels;
    labels.reserve(static_cast<std::size_t>(Error::N));

    if constexpr (requires { typename Error::Pos; }) {
        labels.insert(labels.end(), {"p_e_x_m", "p_e_y_m", "p_e_z_m"});
    }
    if constexpr (requires { typename Error::Vel; }) {
        labels.insert(labels.end(), {"v_e_x_mps", "v_e_y_mps", "v_e_z_mps"});
    }
    if constexpr (requires { typename Error::AttRotVec; }) {
        labels.insert(labels.end(), {"theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"});
    }
    if constexpr (requires { typename Error::GyroB; }) {
        labels.insert(labels.end(),
                      {"gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"});
    }
    if constexpr (requires { typename Error::AccB; }) {
        labels.insert(labels.end(),
                      {"accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"});
    }

    return labels;
}

template<typename Nominal>
std::vector<std::string> nominal_state_labels()
{
    std::vector<std::string> labels;

    if constexpr (requires { typename Nominal::Pos; }) {
        labels.insert(labels.end(), {"p_e_x_m", "p_e_y_m", "p_e_z_m"});
    }
    if constexpr (requires { typename Nominal::Vel; }) {
        labels.insert(labels.end(), {"v_e_x_mps", "v_e_y_mps", "v_e_z_mps"});
    }
    if constexpr (requires { typename Nominal::AttQuat; }) {
        labels.insert(labels.end(), {"q_b2e_w", "q_b2e_x", "q_b2e_y", "q_b2e_z"});
    }
    if constexpr (requires { typename Nominal::GyroB; }) {
        labels.insert(labels.end(),
                      {"gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"});
    }
    if constexpr (requires { typename Nominal::AccB; }) {
        labels.insert(labels.end(),
                      {"accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"});
    }

    return labels;
}

template<typename Error, typename Vector>
void append_state_error_values(std::vector<double>& row, const Vector& values)
{
    if constexpr (requires { typename Error::Pos; }) {
        append_segment_values<typename Error::Pos>(row, values);
    }
    if constexpr (requires { typename Error::Vel; }) {
        append_segment_values<typename Error::Vel>(row, values);
    }
    if constexpr (requires { typename Error::AttRotVec; }) {
        append_segment_values<typename Error::AttRotVec>(row, values);
    }
    if constexpr (requires { typename Error::GyroB; }) {
        append_segment_values<typename Error::GyroB>(row, values);
    }
    if constexpr (requires { typename Error::AccB; }) {
        append_segment_values<typename Error::AccB>(row, values);
    }
}

template<typename Nominal, typename Vector>
void append_nominal_state_values(std::vector<double>& row, const Vector& values)
{
    if constexpr (requires { typename Nominal::Pos; }) {
        append_segment_values<typename Nominal::Pos>(row, values);
    }
    if constexpr (requires { typename Nominal::Vel; }) {
        append_segment_values<typename Nominal::Vel>(row, values);
    }
    if constexpr (requires { typename Nominal::AttQuat; }) {
        append_segment_values<typename Nominal::AttQuat>(row, values);
    }
    if constexpr (requires { typename Nominal::GyroB; }) {
        append_segment_values<typename Nominal::GyroB>(row, values);
    }
    if constexpr (requires { typename Nominal::AccB; }) {
        append_segment_values<typename Nominal::AccB>(row, values);
    }
}

template<typename Error, typename Matrix>
void append_state_error_sigmas(std::vector<double>& row, const Matrix& covariance)
{
    if constexpr (requires { typename Error::Pos; }) {
        append_segment_sigma_values<typename Error::Pos>(row, covariance);
    }
    if constexpr (requires { typename Error::Vel; }) {
        append_segment_sigma_values<typename Error::Vel>(row, covariance);
    }
    if constexpr (requires { typename Error::AttRotVec; }) {
        append_segment_sigma_values<typename Error::AttRotVec>(row, covariance);
    }
    if constexpr (requires { typename Error::GyroB; }) {
        append_segment_sigma_values<typename Error::GyroB>(row, covariance);
    }
    if constexpr (requires { typename Error::AccB; }) {
        append_segment_sigma_values<typename Error::AccB>(row, covariance);
    }
}

inline void append_prefixed_labels(std::vector<std::string>& header,
                                   const std::string& prefix,
                                   const std::vector<std::string>& labels)
{
    for (const auto& label : labels) {
        header.push_back(prefix + label);
    }
}

inline void append_triangular_covariance_header(std::vector<std::string>& header,
                                                const std::vector<std::string>& labels)
{
    for (std::size_t row = 0; row < labels.size(); ++row) {
        for (std::size_t col = row; col < labels.size(); ++col) {
            header.push_back("P_" + labels[row] + "__" + labels[col]);
        }
    }
}

template<typename Matrix>
void append_triangular_covariance_values(std::vector<double>& row, const Matrix& covariance)
{
    for (int r = 0; r < Matrix::RowsAtCompileTime; ++r) {
        for (int c = r; c < Matrix::ColsAtCompileTime; ++c) {
            row.push_back(covariance(r, c));
        }
    }
}

template<typename Error>
std::vector<std::string> state_error_header(const std::string& value_prefix,
                                            CovarianceLogMode covariance_mode)
{
    std::vector<std::string> labels = state_error_labels<Error>();
    std::vector<std::string> header{"time_s"};
    header.reserve(1U + (2U * labels.size()) + (labels.size() * (labels.size() + 1U) / 2U));
    append_prefixed_labels(header, value_prefix, labels);
    append_prefixed_labels(header, "sigma_", labels);
    if (covariance_mode == CovarianceLogMode::Triangular) {
        append_triangular_covariance_header(header, labels);
    }
    return header;
}

template<typename Nominal, typename Error>
std::vector<std::string> nominal_state_with_covariance_header(CovarianceLogMode covariance_mode)
{
    std::vector<std::string> nominal_labels = nominal_state_labels<Nominal>();
    std::vector<std::string> error_labels = state_error_labels<Error>();
    std::vector<std::string> header{"time_s"};
    header.reserve(1U + nominal_labels.size() + error_labels.size() +
                   (error_labels.size() * (error_labels.size() + 1U) / 2U));
    header.insert(header.end(), nominal_labels.begin(), nominal_labels.end());
    append_prefixed_labels(header, "sigma_", error_labels);
    if (covariance_mode == CovarianceLogMode::Triangular) {
        append_triangular_covariance_header(header, error_labels);
    }
    return header;
}

template<typename Error, typename Vector, typename Matrix>
std::vector<double> state_error_row(const navkit::core::Time_t time_s,
                                    const Vector& values,
                                    const Matrix& covariance,
                                    CovarianceLogMode covariance_mode)
{
    std::vector<double> row;
    row.reserve(static_cast<std::size_t>(1 + (2 * Error::N) + ((Error::N * (Error::N + 1)) / 2)));
    row.push_back(time_s);
    append_state_error_values<Error>(row, values);
    append_state_error_sigmas<Error>(row, covariance);
    if (covariance_mode == CovarianceLogMode::Triangular) {
        append_triangular_covariance_values(row, covariance);
    }
    return row;
}

template<typename Nominal, typename Error, typename State, typename Matrix>
std::vector<double> nominal_state_with_covariance_row(const navkit::core::Time_t time_s,
                                                      const State& state,
                                                      const Matrix& covariance,
                                                      CovarianceLogMode covariance_mode)
{
    constexpr int covariance_dimension = Error::N;
    std::vector<double> row;
    row.reserve(
        static_cast<std::size_t>(1 + Nominal::N + covariance_dimension +
                                 ((covariance_dimension * (covariance_dimension + 1)) / 2)));
    row.push_back(time_s);
    append_nominal_state_values<Nominal>(row, state);
    append_state_error_sigmas<Error>(row, covariance);
    if (covariance_mode == CovarianceLogMode::Triangular) {
        append_triangular_covariance_values(row, covariance);
    }
    return row;
}

} // namespace navkit::io::detail
