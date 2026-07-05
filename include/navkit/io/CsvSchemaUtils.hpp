// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <string>
#include <vector>

namespace navkit::io::detail
{

inline void
append_matrix_header(std::vector<std::string>& header, const std::string& name, int rows, int cols)
{
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            header.push_back(name + "_" + std::to_string(r) + "_" + std::to_string(c));
        }
    }
}

template<typename Matrix>
void append_matrix_values(std::vector<double>& row, const Matrix& matrix)
{
    for (int r = 0; r < Matrix::RowsAtCompileTime; ++r) {
        for (int c = 0; c < Matrix::ColsAtCompileTime; ++c) {
            row.push_back(matrix(r, c));
        }
    }
}

} // namespace navkit::io::detail
