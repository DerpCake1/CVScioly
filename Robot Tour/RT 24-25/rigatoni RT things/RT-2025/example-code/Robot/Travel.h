#ifndef TRAVEL_H
#define TRAVEL_H

#include <Arduino.h>
#include "Robot.h"

struct Command
{
    String action;
    double value;
};

class Travel
{
private:
    Robot _robot;
    const Command *_commands;
    uint8_t _commandCount;

public:
    Travel() : _robot(), _commands(nullptr), _commandCount(0) {}

    void setCommands(const Command *commands)
    {
        _commands = commands;
        _commandCount = 0;
        logger.info("New command sequence loaded");
    }

    void prepareSteppers()
    {
        logger.info("Preparing steppers");
        _robot.turnOnSteppers();
    }

    void shutdownSteppers()
    {
        logger.info("Shutting down steppers");
        _robot.turnOffSteppers();
    }

    void execute()
    {
        if (_commands == nullptr)
        {
            logger.error("No commands loaded");
            return;
        }

        logger.info("Starting command sequence in 2 seconds");
        delay(2000);

        while (_commands[_commandCount].action != "")
        {
            Command cmd = _commands[_commandCount];
            logger.info("Executing command %d: type=%s, value=%D",
                        _commandCount, cmd.action, cmd.value);

            if (cmd.action == "move")
            {
                _robot.move(static_cast<long>(cmd.value));
            }
            else if (cmd.action == "turn")
            {
                _robot.turn(cmd.value);
            }
            else if (cmd.action == "stop")
            {
                _robot.stop(static_cast<unsigned long>(cmd.value));
            }
            else
            {
                logger.error("Unknown command: %s", cmd.action);
                return;
            }
            _commandCount++;
        }
        logger.info("Command sequence completed");
        shutdownSteppers();
    }
};

#endif
