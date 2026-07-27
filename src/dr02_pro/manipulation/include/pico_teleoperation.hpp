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

namespace deep_robotics::dr02_pro {

class PicoTeleoperationController {
public:
    static constexpr int kDof = 31;

    struct Command {
        std::array<float, kDof> position{};
        std::array<float, kDof> velocity{};
        bool ready = false;
    };

    PicoTeleoperationController();
    ~PicoTeleoperationController();

    PicoTeleoperationController(const PicoTeleoperationController&) = delete;
    PicoTeleoperationController& operator=(const PicoTeleoperationController&) = delete;

    /**
     * @brief Initialize the Pico SDK callback.
     * @return Return code from PXREAInit, or 0 when already started.
     */
    int Start();

    /**
     * @brief Deinitialize the Pico SDK callback.
     */
    void Stop();

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
    Command Update(const std::array<float, kDof>& current);

    /**
     * @brief Record total state-control time for the periodic diagnostic.
     * @param milliseconds Time from state update through command publication.
     */
    void RecordControlLoopDuration(double milliseconds);

    /**
     * @brief Reset hand/head anchors to the current robot state.
     */
    void ResetTargetsToCurrent(const std::array<float, kDof>& current);

    /**
     * @brief Whether the Pico server is currently connected.
     */
    bool IsConnected() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace deep_robotics::dr02_pro
