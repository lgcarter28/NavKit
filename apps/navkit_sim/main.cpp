// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/SelectedConfig.hpp"
#include "navkit/app_support/AppRunner.hpp"
#include "navkit/app_support/ConfigDescription.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace fs = std::filesystem;

int main(int argc, char** argv)
{
    try {
        using AppConfig = navkit::selected_config::Config;

        if (argc > 1 && std::string_view(argv[1]) == "--describe-config") {
            return navkit::app_support::describe_compiletime_config<AppConfig>(std::cout);
        }

        const fs::path config_path =
            (argc > 1) ? fs::path(argv[1])
                       : fs::path("config/runtime/navkit_sim/stationary_gnss.json");
        return navkit::app_support::run_selected_app<AppConfig>(config_path);
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "navkit_sim error: %s\n", e.what());
        return 1;
    }
}
