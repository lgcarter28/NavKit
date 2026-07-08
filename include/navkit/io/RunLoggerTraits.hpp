// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/LogProductPolicy.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace navkit::io::detail
{

template<typename Payload, typename Product>
inline constexpr bool product_logs_payload_v = LogProductPolicy<Product, Payload>;

template<typename Payload, typename... Products>
inline constexpr std::size_t matching_product_count_v =
    (std::size_t{0} + ... +
     (product_logs_payload_v<Payload, Products> ? std::size_t{1} : std::size_t{0}));

template<typename Product>
std::filesystem::path metadata_path(const std::filesystem::path& output_dir)
{
    const auto manifest = Product::manifest_entry();
    if (!manifest.contains("manifest") || !manifest.at("manifest").is_string()) {
        throw std::runtime_error("Log product manifest entry must include a manifest file name");
    }

    return output_dir / manifest.at("manifest").template get<std::string>();
}

} // namespace navkit::io::detail
