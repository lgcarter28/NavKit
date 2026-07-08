// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/RunLoggerTraits.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace navkit::io
{

template<typename... LogProducts>
class RunLogger
{
public:
    static_assert(sizeof...(LogProducts) > 0, "RunLogger requires at least one log product.");

    template<typename Payload>
    static constexpr std::size_t matching_product_count_v =
        detail::matching_product_count_v<Payload, LogProducts...>;

    RunLogger(std::filesystem::path output_dir, std::string run_name, nlohmann::json config)
        : m_output_dir(std::move(output_dir))
        , m_run_name(std::move(run_name))
        , m_config(std::move(config))
    {
        std::filesystem::create_directories(m_output_dir);
        open_products(std::make_index_sequence<sizeof...(LogProducts)>{});
    }

    template<typename Product>
    Product& product()
    {
        return std::get<Product>(m_products);
    }

    template<typename Payload>
        requires(matching_product_count_v<Payload> == 1U)
    void log(const Payload& payload)
    {
        log_payload(payload, std::make_index_sequence<sizeof...(LogProducts)>{});
    }

    void close()
    {
        if (m_closed) {
            return;
        }

        flush_products(std::make_index_sequence<sizeof...(LogProducts)>{});
        write_metadata_files(std::make_index_sequence<sizeof...(LogProducts)>{});
        write_json_file(m_output_dir / "run_manifest.json", run_manifest());

        m_closed = true;
    }

    const std::filesystem::path& output_dir() const
    {
        return m_output_dir;
    }

private:
    template<std::size_t... Is>
    void open_products(std::index_sequence<Is...>)
    {
        (std::get<Is>(m_products).open(m_output_dir), ...);
    }

    template<std::size_t... Is>
    void flush_products(std::index_sequence<Is...>)
    {
        (std::get<Is>(m_products).flush(), ...);
    }

    template<std::size_t... Is>
    void write_metadata_files(std::index_sequence<Is...>) const
    {
        (write_product_metadata<std::tuple_element_t<Is, ProductTuple>>(std::get<Is>(m_products)),
         ...);
    }

    template<typename Product>
    void write_product_metadata(const Product& product) const
    {
        write_json_file(detail::metadata_path<Product>(m_output_dir), product.metadata());
    }

    template<typename Payload, std::size_t... Is>
    void log_payload(const Payload& payload, std::index_sequence<Is...>)
    {
        (log_payload_if_supported<std::tuple_element_t<Is, ProductTuple>>(std::get<Is>(m_products),
                                                                          payload),
         ...);
    }

    template<typename Product, typename Payload>
    static void log_payload_if_supported(Product& product, const Payload& payload)
    {
        if constexpr (LogProductPolicy<Product, Payload>) {
            product.log(payload);
        }
    }

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
        nlohmann::json logs = nlohmann::json::object();
        append_manifest_entries(logs, std::make_index_sequence<sizeof...(LogProducts)>{});

        return {{"run_name", m_run_name}, {"config", m_config}, {"logs", logs}};
    }

    template<std::size_t... Is>
    static void append_manifest_entries(nlohmann::json& logs, std::index_sequence<Is...>)
    {
        (append_manifest_entry<std::tuple_element_t<Is, ProductTuple>>(logs), ...);
    }

    template<typename Product>
    static void append_manifest_entry(nlohmann::json& logs)
    {
        logs[std::string(Product::LogKey)] = Product::manifest_entry();
    }

    using ProductTuple = std::tuple<LogProducts...>;

    std::filesystem::path m_output_dir;
    std::string m_run_name;
    nlohmann::json m_config;
    ProductTuple m_products;

    bool m_closed = false;
};

} // namespace navkit::io
