// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>
#include <filesystem>

namespace navkit::io
{

template<typename Candidate>
concept LoggerPolicy = requires(Candidate& logger, const Candidate& const_logger) {
    { logger.close() } -> std::same_as<void>;
    { const_logger.output_dir() } -> std::same_as<const std::filesystem::path&>;
};

template<typename Candidate, typename Payload>
concept LoggerPayloadPolicy =
    LoggerPolicy<Candidate> && requires(Candidate& logger, const Payload& payload) {
        { logger.log(payload) } -> std::same_as<void>;
    };

template<typename Candidate, typename Product>
concept LoggerProductAccessPolicy = LoggerPolicy<Candidate> && requires(Candidate& logger) {
    { logger.template product<Product>() } -> std::same_as<Product&>;
};

} // namespace navkit::io
