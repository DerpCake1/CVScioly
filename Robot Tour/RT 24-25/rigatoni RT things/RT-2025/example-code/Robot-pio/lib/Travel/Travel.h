#ifndef TRAVEL_H
#define TRAVEL_H

#include <Arduino.h>
#include "Robot.h"
#include "Logger.h"
#include "Commands.h"
#include "Modes.h"

#include <vector>
#include <unordered_map>

class Travel
{
private:
    Robot _robot;
    const CommandSequence *_commands;
    unsigned long _totalStopTime;
    unsigned long _dryRunStopTime;
    bool _dryRun;
    bool _useIMU;
    Mode _mode;
    static const unsigned long MAX_STOP_TIME = 60000;

public:
    Travel(bool useIMU = true) : _robot(),
                                 _commands(nullptr),
                                 _totalStopTime(0),
                                 _dryRunStopTime(0),
                                 _dryRun(false),
                                 _useIMU(useIMU),
                                 _mode(Mode::TEST)
    {
        _robot.setUseIMU(useIMU);
    }

    void loadCommandSequence(const CommandSequence &commands)
    {
        _commands = &commands;
        logger.info("New command sequence loaded");
    }

    void setTotalStopTime(unsigned long duration)
    {
        _totalStopTime = min(duration, MAX_STOP_TIME);
    }

    void setMode(Mode mode)
    {
        _mode = mode;
    }

    void initialize()
    {
        logger.info("Initializing motion system (steppers and IMU)");
        logger.info("Use IMU: %T", _useIMU);
        _robot.turnOnSteppers();
        if (_mode != Mode::DRY_RUN)
            _robot.startLasers();
        _robot.startIMU();
    }

    void close()
    {
        _robot.turnOffSteppers();
    }

    void setDryRun(bool dryRun = false)
    {
        _dryRun = dryRun;
        if (_dryRun)
            logger.info("Set DRY_RUN");
    }

    void execute()
    {
        if (_mode != Mode::TEST)
        {
            _robot.stopLasers();
        }

        if (_commands == nullptr || _commands->getCommands().empty())
        {
            logger.error("No commands loaded");
            return;
        }

        logger.info("Starting command sequence in 2 seconds");
        logger.lcdPrint("Starting\nin 2 s", COLOR_YELLOW);

        delay(2000);

        logger.lcdPrint("Running", COLOR_RED);

        unsigned long startTime = millis();

        const std::vector<Command> &commands = _commands->getCommands();
        for (size_t i = 0; i < commands.size(); i++)
        {
            const Command &cmd = commands[i];
            logger.info("--------------------------------");
            logger.info("Executing command %d: type=%s, value=%D",
                        i, cmd.action.c_str(), cmd.value);

            if (cmd.action == "move")
            {
                _robot.move(static_cast<long>(cmd.value));
            }
            else if (cmd.action == "turn")
            {
                if (_dryRun)
                {
                    // In dry run mode, use predefined stop times based on angle
                    auto it = DRY_RUN_STOP_TIMES.find(abs(cmd.value));
                    if (it != DRY_RUN_STOP_TIMES.end())
                    {
                        // _robot.stop(it->second);
                        _dryRunStopTime += it->second;
                    }
                    else
                    {
                        // Find the closest angle in the map
                        double closestAngle = 45; // Initialize with smallest angle
                        double minDiff = 180;     // Initialize with maximum possible difference
                        double targetAngle = abs(cmd.value);

                        for (const auto &pair : DRY_RUN_STOP_TIMES)
                        {
                            double diff = abs(pair.first - targetAngle);
                            if (diff < minDiff)
                            {
                                minDiff = diff;
                                closestAngle = pair.first;
                            }
                        }

                        // _robot.stop(DRY_RUN_STOP_TIMES.at(closestAngle));
                        _dryRunStopTime += DRY_RUN_STOP_TIMES.at(closestAngle);
                    }
                }
                else
                {
                    _robot.turn(cmd.value);
                }
            }
            else if (cmd.action == "stop")
            {
                if (_dryRun)
                {
                    _dryRunStopTime += static_cast<unsigned long>(cmd.value);
                }
                else
                {
                    _robot.stop(static_cast<unsigned long>(cmd.value));
                }
            }
            else
            {
                logger.error("Unknown command: %s", cmd.action.c_str());
                logger.lcdPrint("Unknown command");
                return;
            }

            // If this isn't the last command, delay for a portion of the total stop time
            if (i < commands.size() - 1)
            {
                unsigned long stopTime = _totalStopTime / (commands.size() - 1);
                if (_dryRun)
                {
                    _dryRunStopTime += stopTime;
                }
                else
                {
                    _robot.stop(stopTime);
                }
            }
        }
        unsigned long totalTime = millis() - startTime;
        if (_dryRun)
        {
            totalTime += _dryRunStopTime;
            _dryRunStopTime = 0;
        }
        float totalSeconds = totalTime / 1000.0;
        logger.lcdSet(COLOR_GREEN);
        logger.lcdPrintf("RunTime:\n%.2f s", totalSeconds);
        logger.info("================================================");
        logger.info("Total extra stop time: %F seconds", _totalStopTime / 1000.0);
        logger.info("Total run time %F seconds", totalSeconds);
        logger.info("================================================");
    }
};

#endif