// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/io/CsvWriter.hpp"
#include "test_main.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace navkit::io::test
{

namespace
{

std::string read_file(const std::filesystem::path& path)
{
    std::ifstream file(path);
    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

} // namespace

TEST_CASE("CsvWriter writes header and rows with comma separation")
{
    const auto path = std::filesystem::temp_directory_path() / "navkit_csv_writer_test.csv";
    std::filesystem::remove(path);

    {
        CsvWriter writer(path, {"time_s", "value"});
        writer.write_row(1.25, 42);
        writer.flush();
    }

    const std::string contents = read_file(path);
    CHECK(contents.find("time_s,value\n") == 0U);
    CHECK(contents.find("1.25,42\n") != std::string::npos);

    std::filesystem::remove(path);
}

TEST_CASE("CsvWriter reports invalid output paths")
{
    const auto directory_path = std::filesystem::temp_directory_path();

    CHECK_THROWS_AS(CsvWriter(directory_path, {"value"}), std::runtime_error);
}

} // namespace navkit::io::test
