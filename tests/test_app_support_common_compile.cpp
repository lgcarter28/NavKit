// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/MissionApp.hpp"
#include "navkit/app_support/execution/ExecutionTarget.hpp"
#include "navkit/app_support/execution/MissionAdapter.hpp"
#include "navkit/app_support/mission/MissionRuntime.hpp"

int main()
{
    const navkit::app_support::MissionRuntime mission{};
    return mission.is_valid() ? 1 : 0;
}
