#include <Arduino.h>
#include <string.h>

#include <ezButton.h>
#include "Travel.h"
#include "Logger.h"
#include "config.h"
#include "Modes.h"

// Define pin macros
#define MODE_BUTTON 12
#define START_BUTTON 13

// Initialize logger and travel
Logger logger;
Travel travel(useIMU); // Pass IMU configuration to Travel

// Create button objects using macros
ezButton modeButton(MODE_BUTTON);
ezButton startButton(START_BUTTON);

// Initialize mode
Mode currentMode = Mode::TEST;
String modeName = "TEST";
bool isModeSelected = false; // Add flag to track if mode has been selected
CommandSequence *sequence = &testSequence;

void setup()
{
    modeButton.setDebounceTime(50);
    startButton.setDebounceTime(50);
    logger.info("Setup complete, press the mode button");
    logger.lcdPrint("Press\nmode\nbutton");
}

void loop()
{
    // Must call these every loop for proper debouncing
    modeButton.loop();
    startButton.loop();

    // Toggle mode when mode button is released
    if (modeButton.isReleased())
    {
        currentMode = static_cast<Mode>((currentMode + 1) % 3);
        switch (currentMode)
        {
        case Mode::TEST:
            modeName = "TEST";
            logger.lcdSet(COLOR_GREEN);
            sequence = &testSequence;
            travel.setDryRun(false);
            break;
        case Mode::TRACK_RUN:
            modeName = "TRACKRUN";
            logger.lcdSet(COLOR_BLUE);
            sequence = &runSequence;
            travel.setDryRun(false);
            break;
        case Mode::DRY_RUN:
            modeName = "DRY RUN";
            logger.lcdSet(COLOR_YELLOW);
            sequence = &runSequence;
            travel.setDryRun(useIMU ? true : false);
            break;
        }
        travel.setMode(currentMode);
        logger.info("Mode changed to: %s", modeName);
        logger.lcdPrintf("MODE:\n%s", modeName);

        if (!isModeSelected)
        {
            travel.initialize();
            isModeSelected = true;
        }
    }

    // Execute the appropriate sequence when start button is released
    // Only allow start if mode was selected
    if (isModeSelected && startButton.isReleased())
    {
        travel.setTotalStopTime(totalStopTime); // Set the total stop time
        travel.loadCommandSequence(*sequence);
        logger.info(">>> STARTING SEQUENCE in %s mode", modeName);
        travel.execute();
        travel.close();

        isModeSelected = false; // Reset the flag after execution
        logger.info("<<< SEQUENCE COMPLETED");
    }
}
