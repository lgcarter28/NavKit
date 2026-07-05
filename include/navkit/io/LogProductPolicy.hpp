// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace navkit::io
{

template<typename Candidate, typename Payload>
concept LogProductPolicy =
    requires(Candidate product, const std::filesystem::path& output_dir, const Payload& payload) {
        { product.open(output_dir) } -> std::same_as<void>;
        { product.log(payload) } -> std::same_as<void>;
        { product.flush() } -> std::same_as<void>;
        { product.metadata() } -> std::same_as<nlohmann::json>;
        { Candidate::manifest_entry() } -> std::same_as<nlohmann::json>;
    };

} // namespace navkit::io
