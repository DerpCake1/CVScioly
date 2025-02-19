#include <ezButton.h>
#include "Travel.h"
#include "Logger.h"

// Define pin macros
#define MODE_BUTTON 12
#define START_BUTTON 13

Logger logger;

Travel travel;

// Create button objects using macros
ezButton modeButton(MODE_BUTTON);
ezButton startButton(START_BUTTON);

// Define the sequence of commands with a nullptr as the terminator
Command run[] = {
    {"move", 500},
    {"stop", 1000},
    {"move", -500},
    {"stop", 1000},
    {"turn", -90},
    {"stop", 1000},
    {"turn", 90},
    {"", 0}};

Command test[] = {
    {"move", 1000},
    {"move", -1000},
    {"turn", -90},
    {"turn", 90},
    {"", 0}};

// Add mode tracking
bool isTestMode = true;      // true = test mode, false = run mode
bool isModeSelected = false; // Add flag to track if mode has been selected

void setup()
{
    modeButton.setDebounceTime(50);
    startButton.setDebounceTime(50);
}

void loop()
{
    // Must call these every loop for proper debouncing
    modeButton.loop();
    startButton.loop();

    // Toggle mode when mode button is released
    if (modeButton.isReleased())
    {
        isTestMode = !isTestMode;
        logger.info("Mode changed to: %s", isTestMode ? "TEST" : "RUN");
        travel.prepareSteppers();
        isModeSelected = true;
    }

    // Execute the appropriate sequence when start button is released
    if (isModeSelected && startButton.isReleased()) // Only allow start if mode was selected
    {
        logger.info("Starting sequence in %s mode", isTestMode ? "TEST" : "RUN");

        travel.setCommands(isTestMode ? test : run);
        travel.execute();
        isModeSelected = false; // Reset the flag after execution
        logger.info("Sequence completed");
    }
}
