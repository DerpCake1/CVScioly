#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>
#include <vector>

struct Command
{
    String action;
    double value;
};

class CommandSequence
{
private:
    std::vector<Command> commands;

public:
    CommandSequence &move(long distance = 500)
    {
        commands.push_back({"move", static_cast<double>(distance)});
        return *this;
    }

    CommandSequence &turn(double degrees = 90)
    {
        commands.push_back({"turn", degrees});
        return *this;
    }

    CommandSequence &stop(unsigned long duration)
    {
        commands.push_back({"stop", static_cast<double>(duration)});
        return *this;
    }

    CommandSequence &left(double degrees = 90)
    {
        return turn(-degrees);
    }

    CommandSequence &right(double degrees = 90)
    {
        return turn(degrees);
    }

    CommandSequence &forward(long distance = 500)
    {
        return move(distance);
    }

    CommandSequence &backward(long distance = 500)
    {
        return move(-distance);
    }

    // Aliases for common commands
    CommandSequence &l(double degrees = 90) { return left(degrees); }
    CommandSequence &r(double degrees = 90) { return right(degrees); }
    CommandSequence &f(long distance = 500) { return forward(distance); }
    CommandSequence &b(long distance = 500) { return backward(distance); }

    const std::vector<Command> &getCommands() const
    {
        return commands;
    }
};

#endif
