// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::io
{

class CsvWriter
{
public:
    CsvWriter() = default;

    CsvWriter(const std::filesystem::path& path, std::initializer_list<std::string_view> header)
    {
        open(path, header);
    }

    CsvWriter(const std::filesystem::path& path, const std::vector<std::string>& header)
    {
        open(path, header);
    }

    void open(const std::filesystem::path& path, std::initializer_list<std::string_view> header)
    {
        m_file.open(path);
        if (!m_file) {
            throw std::runtime_error("Failed to open CSV file: " + path.string());
        }

        m_file << std::setprecision(17);
        write_header(header);
    }

    void open(const std::filesystem::path& path, const std::vector<std::string>& header)
    {
        m_file.open(path);
        if (!m_file) {
            throw std::runtime_error("Failed to open CSV file: " + path.string());
        }

        m_file << std::setprecision(17);
        write_header(header);
    }

    template<typename... Values>
    void write_row(const Values&... values)
    {
        bool first = true;
        ((write_value(values, first)), ...);
        m_file << '\n';
    }

    bool is_open() const
    {
        return m_file.is_open();
    }

    void flush()
    {
        m_file.flush();
    }

private:
    void write_header(std::initializer_list<std::string_view> header)
    {
        bool first = true;
        for (const auto& name : header) {
            if (!first) {
                m_file << ',';
            }
            first = false;
            m_file << name;
        }
        m_file << '\n';
    }

    void write_header(const std::vector<std::string>& header)
    {
        bool first = true;
        for (const auto& name : header) {
            if (!first) {
                m_file << ',';
            }
            first = false;
            m_file << name;
        }
        m_file << '\n';
    }

    template<typename Value>
    void write_value(const Value& value, bool& first)
    {
        if (!first) {
            m_file << ',';
        }
        first = false;
        m_file << value;
    }

    std::ofstream m_file;
};

} // namespace navkit::io
