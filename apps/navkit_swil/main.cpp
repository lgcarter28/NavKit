// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/SelectedConfig.hpp"
#include "navkit/app_support/MissionApp.hpp"
#include "navkit/app_support/config/ConfigDescription.hpp"
#include "navkit/swil/SwilMissionAdapterFactory.hpp"

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

        if (argc <= 1) {
            std::fprintf(stderr,
                         "navkit_swil requires a resolved runtime JSON file; use "
                         "python tools/run_scenario.py for component-linked scenarios.\n");
            return 2;
        }

        const fs::path config_path{argv[1]};
        return navkit::app_support::MissionApp<AppConfig>::template run<
            navkit::swil::SwilMissionAdapterFactory>(config_path);
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "navkit_swil error: %s\n", e.what());
        return 1;
    }
}
