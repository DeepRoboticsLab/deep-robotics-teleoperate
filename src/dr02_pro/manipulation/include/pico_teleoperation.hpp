/**
 * @file pico_teleoperation.hpp
 * @brief Pico teleoperation controller for DR02 Pro upper-body control.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <array>
#include <memory>

#include "pico_data_receiver.hpp"

namespace deep_robotics::dr02_pro {

class PicoTeleoperationController {
public:
    static constexpr int kDof = 31;

    struct Command {
        std::array<float, kDof> position{};
        std::array<float, kDof> velocity{};
        bool left_primary_pressed = false;
        bool left_secondary_pressed = false;
        bool ready = false;
    };

    /**
     * @brief Construct the Pico controller.
     * @param periodic_log_enabled Print the periodic diagnostic when true.
     */
    explicit PicoTeleoperationController(
        bool periodic_log_enabled = false);
    ~PicoTeleoperationController();

    PicoTeleoperationController(const PicoTeleoperationController&) = delete;
    PicoTeleoperationController& operator=(const PicoTeleoperationController&) = delete;

    /** Reset controller-local session and recovery state. */
    void Reset();

    /**
     * @brief Seed teleoperation targets from the current robot joint state.
     * @param current Current 31-DoF joint positions.
     */
    void SeedFromCurrent(const std::array<float, kDof>& current);

    /**
     * @brief Run one teleoperation update.
     * @param current Current 31-DoF joint positions.
     * @return Smoothed 31-DoF target command.
     */
    Command Update(const std::array<float, kDof>& current,
                   const PicoState& pico);

    /**
     * @brief Record total state-control time for the periodic diagnostic.
     * @param milliseconds Time from state update through command publication.
     */
    void RecordControlLoopDuration(double milliseconds);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace deep_robotics::dr02_pro
