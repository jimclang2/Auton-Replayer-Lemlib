#include "position_replay.h"
#include "robot_config.h"
#include <cstdio>
#include <cmath>

// Global instance
PositionReplay positionReplay;

// ==================== Helper Functions ====================

uint8_t PositionReplay::packButtons() {
    uint8_t buttons = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) buttons |= (1 << BTN_R1);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) buttons |= (1 << BTN_R2);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) buttons |= (1 << BTN_L1);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) buttons |= (1 << BTN_L2);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))  buttons |= (1 << BTN_X);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))  buttons |= (1 << BTN_A);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))  buttons |= (1 << BTN_B);
    return buttons;
}

bool PositionReplay::wasPressed(uint8_t current, uint8_t prev, uint8_t bit) {
    return (current & (1 << bit)) && !(prev & (1 << bit));
}

bool PositionReplay::isSDCardInserted() const {
    FILE* test = fopen("/usd/.", "r");
    if (test) {
        fclose(test);
        return true;
    }
    return false;
}

bool PositionReplay::checkEmergencyStop() {
    // Left + Right arrow combo for emergency stop
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && 
           master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
}

void PositionReplay::displayCountdown(int secondsRemaining) {
    // Display on brain screen
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(180, 80, 300, 160);
    pros::screen::set_pen(pros::c::COLOR_YELLOW);
    pros::screen::print(pros::E_TEXT_LARGE, 200, 100, "Starting in");
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 230, 130, "%d", secondsRemaining);
    
    // Display on controller
    master.print(0, 0, "Starting in %d...  ", secondsRemaining);
    master.rumble(".");
}

void PositionReplay::abortPlayback() {
    _abortRequested = true;
}

// ==================== Recording ====================

void PositionReplay::startRecording() {
    // Check SD card
    if (!isSDCardInserted()) {
        master.print(0, 0, "WARNING: No SD Card!");
        master.rumble("---");
        pros::delay(1000);
        master.print(0, 0, "Recording anyway...");
    }
    
    // Countdown
    if (countdownDuration > 0) {
        int seconds = countdownDuration / 1000;
        for (int i = seconds; i > 0; i--) {
            displayCountdown(i);
            pros::delay(1000);
        }
    }
    
    recording.clear();
    
    // Reserve memory (~3 minutes at 10 samples/sec = ~1800 frames)
    try {
        recording.reserve(2000);
    } catch (...) {
        master.print(0, 0, "MEM RESERVE FAILED!");
        master.rumble("---");
    }
    
    // CRITICAL: Reset odometry to (0, 0, 0) for consistent reference
    chassis.setPose(0, 0, 0);
    
    recordStartTime = pros::micros();
    lastRecordTime = 0;
    _isRecording = true;
    prevButtons = 0;
    
    // Boost task priority for consistent timing
    pros::Task::current().set_priority(TASK_PRIORITY_MAX - 1);
    
    master.print(0, 0, "RECORDING (POS)... ");
    master.rumble("-");
    
    drawStatusIndicator();
}

void PositionReplay::stopRecording(bool saveToSD) {
    _isRecording = false;
    
    // Restore normal priority
    pros::Task::current().set_priority(TASK_PRIORITY_DEFAULT);
    
    master.print(0, 0, "STOPPED: %d pts   ", recording.size());
    master.rumble(".");
    
    if (saveToSD) {
        if (!isSDCardInserted()) {
            master.print(1, 0, "NO SD CARD!        ");
            master.rumble("---");
        } else if (this->saveToSD()) {
            master.print(1, 0, "SAVED TO SD!       ");
        } else {
            master.print(1, 0, "SD SAVE FAILED!    ");
        }
    }
    
    drawStatusIndicator();
}

void PositionReplay::recordFrame() {
    if (!_isRecording) return;
    
    uint64_t currentTime = pros::micros() - recordStartTime;
    uint64_t currentTimeMs = currentTime / 1000;
    
    // Only record at configured interval (default 100ms)
    if (currentTimeMs - (lastRecordTime / 1000) < recordingInterval) {
        return;
    }
    
    lastRecordTime = currentTime;
    
    // Get current pose from LemLib odometry
    lemlib::Pose pose = chassis.getPose();
    
    // Get current button state
    uint8_t currentButtons = packButtons();
    
    // Determine if any action is happening at this frame
    bool actionOccurred = false;
    
    // Check for button state changes (pneumatic toggles)
    if (wasPressed(currentButtons, prevButtons, BTN_X) ||
        wasPressed(currentButtons, prevButtons, BTN_A) ||
        wasPressed(currentButtons, prevButtons, BTN_B)) {
        actionOccurred = true;
    }
    
    // Check if motor power is significant (intake/outtake active)
    int intakeVoltage = Intake.get_voltage();
    int outtakeVoltage = Outtake.get_voltage();
    int8_t intakePower = static_cast<int8_t>(intakeVoltage * 127 / 12000);
    int8_t outtakePower = static_cast<int8_t>(outtakeVoltage * 127 / 12000);
    
    // Consider motor action if power is above threshold
    if (std::abs(intakePower) > 10 || std::abs(outtakePower) > 10) {
        actionOccurred = true;
    }
    
    // Build frame
    WaypointFrame frame;
    frame.x = pose.x;
    frame.y = pose.y;
    frame.theta = pose.theta;
    frame.timestamp = currentTime;
    frame.intakePower = intakePower;
    frame.outtakePower = outtakePower;
    frame.buttons = currentButtons;
    frame.hasAction = actionOccurred;
    
    prevButtons = currentButtons;
    
    // Safe push_back
    try {
        recording.push_back(frame);
    } catch (...) {
        master.print(0, 0, "MEMORY FULL!       ");
        stopRecording(true);
        return;
    }
    
    // Blink indicator
    uint32_t elapsedSec = currentTimeMs / 500;
    if (elapsedSec % 2 == 0) {
        pros::screen::set_pen(pros::c::COLOR_CYAN);
    } else {
        pros::screen::set_pen(pros::c::COLOR_DARK_CYAN);
    }
    pros::screen::fill_circle(460, 20, 15);
}

// ==================== Playback ====================

void PositionReplay::executeActions(const WaypointFrame& frame, bool& midScoring, bool& descore, bool& unloader) {
    // Apply motor powers directly
    Intake.move(frame.intakePower);
    Outtake.move(frame.outtakePower);
    
    // Handle pneumatic toggles with edge detection
    static uint8_t lastPlaybackButtons = 0;
    
    if (wasPressed(frame.buttons, lastPlaybackButtons, BTN_X)) {
        midScoring = !midScoring;
        MidScoring.set_value(midScoring);
    }
    if (wasPressed(frame.buttons, lastPlaybackButtons, BTN_A)) {
        descore = !descore;
        Descore.set_value(descore);
    }
    if (wasPressed(frame.buttons, lastPlaybackButtons, BTN_B)) {
        unloader = !unloader;
        Unloader.set_value(unloader);
    }
    
    lastPlaybackButtons = frame.buttons;
}

void PositionReplay::playback() {
    if (recording.empty()) {
        if (!loadFromSD()) {
            master.print(0, 0, "NO RECORDING!      ");
            return;
        }
    }
    
    _isPlaying = true;
    _abortRequested = false;
    
    // Pneumatic states
    bool midScoring = false;
    bool descore = false;
    bool unloader = false;
    
    // Reset pose to starting position (should be 0,0,0)
    chassis.setPose(0, 0, 0);
    pros::delay(50);
    
    // Boost priority
    pros::Task::current().set_priority(TASK_PRIORITY_MAX - 1);
    
    master.print(0, 0, "REPLAYING (<>=STOP)");
    
    // Status indicator
    pros::screen::set_pen(pros::c::COLOR_GREEN);
    pros::screen::fill_circle(460, 20, 15);
    
    size_t frameIndex = 0;
    size_t actionFrameIndex = 0;  // Track which action frames we've processed
    
    // Find waypoints that have actions (we'll pause at these)
    std::vector<size_t> actionFrames;
    for (size_t i = 0; i < recording.size(); i++) {
        if (recording[i].hasAction) {
            actionFrames.push_back(i);
        }
    }
    
    // Main playback loop - move through waypoints
    while (frameIndex < recording.size()) {
        // Emergency stop check
        if (checkEmergencyStop() || _abortRequested) {
            master.print(0, 0, "PLAYBACK ABORTED!  ");
            master.rumble("--");
            break;
        }
        
        WaypointFrame& target = recording[frameIndex];
        
        // Use moveToPose for smooth motion to next waypoint
        // Use async=false to wait for completion, with a reasonable timeout
        int timeout = 1500;  // Max 1.5 second per waypoint
        
        // Calculate distance to determine if we should move or just turn
        lemlib::Pose currentPose = chassis.getPose();
        float dx = target.x - currentPose.x;
        float dy = target.y - currentPose.y;
        float distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance > 1.0f) {
            // Move to the waypoint position with target heading
            chassis.moveToPose(target.x, target.y, target.theta, timeout, {}, false);
        } else if (std::abs(target.theta - currentPose.theta) > 5.0f) {
            // Just turn if we're already at position but wrong heading
            chassis.turnToHeading(target.theta, 500, {}, false);
        }
        
        // Check if this is an action frame - if so, execute actions
        if (target.hasAction) {
            executeActions(target, midScoring, descore, unloader);
            // Brief pause for mechanism action to complete
            pros::delay(50);
        }
        
        // Continue applying motor powers between waypoints
        Intake.move(target.intakePower);
        Outtake.move(target.outtakePower);
        
        // Blink indicator
        uint32_t currentMs = pros::millis();
        if ((currentMs / 500) % 2 == 0) {
            pros::screen::set_pen(pros::c::COLOR_GREEN);
        } else {
            pros::screen::set_pen(pros::c::COLOR_DARK_GREEN);
        }
        pros::screen::fill_circle(460, 20, 15);
        
        frameIndex++;
    }
    
    // Stop all motors
    left_motors.move(0);
    right_motors.move(0);
    Intake.move(0);
    Outtake.move(0);
    
    // Restore priority
    pros::Task::current().set_priority(TASK_PRIORITY_DEFAULT);
    
    _isPlaying = false;
    _abortRequested = false;
    
    master.print(0, 0, "REPLAY COMPLETE!   ");
    
    // Clear indicator
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_circle(460, 20, 15);
}

// ==================== Data Management ====================

void PositionReplay::clearRecording() {
    recording.clear();
    master.print(0, 0, "RECORDING CLEARED  ");
}

uint32_t PositionReplay::getDuration() const {
    if (recording.empty()) return 0;
    return recording.back().timestamp / 1000;  // Convert micros to ms
}

bool PositionReplay::saveToSD() {
    if (!isSDCardInserted()) {
        return false;
    }
    
    FILE* file = fopen(filePath.c_str(), "wb");
    if (!file) {
        return false;
    }
    
    // Write header: magic number + version + frame count
    uint32_t magic = 0x504F5352;  // "POSR" for Position Recording
    uint32_t version = 1;
    uint32_t frameCount = recording.size();
    
    fwrite(&magic, sizeof(uint32_t), 1, file);
    fwrite(&version, sizeof(uint32_t), 1, file);
    fwrite(&frameCount, sizeof(uint32_t), 1, file);
    
    // Write all frames
    for (const auto& frame : recording) {
        fwrite(&frame, sizeof(WaypointFrame), 1, file);
    }
    
    fclose(file);
    return true;
}

bool PositionReplay::loadFromSD() {
    if (!isSDCardInserted()) {
        master.print(0, 0, "NO SD CARD!        ");
        return false;
    }
    
    FILE* file = fopen(filePath.c_str(), "rb");
    if (!file) {
        return false;
    }
    
    // Read header
    uint32_t magic, version, frameCount;
    if (fread(&magic, sizeof(uint32_t), 1, file) != 1 ||
        fread(&version, sizeof(uint32_t), 1, file) != 1 ||
        fread(&frameCount, sizeof(uint32_t), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Verify magic number
    if (magic != 0x504F5352) {
        fclose(file);
        master.print(0, 0, "INVALID FILE!      ");
        return false;
    }
    
    // Sanity check (max ~2000 frames = ~3.3 mins at 10 samples/sec)
    if (frameCount > 5000) {
        fclose(file);
        master.print(0, 0, "FILE TOO LARGE!    ");
        return false;
    }
    
    // Read all frames
    recording.clear();
    recording.resize(frameCount);
    
    for (uint32_t i = 0; i < frameCount; i++) {
        if (fread(&recording[i], sizeof(WaypointFrame), 1, file) != 1) {
            fclose(file);
            recording.clear();
            return false;
        }
    }
    
    fclose(file);
    master.print(0, 0, "LOADED: %d pts     ", frameCount);
    return true;
}

// ==================== Status Display ====================

void PositionReplay::drawStatusIndicator() {
    // Draw status in top-right corner
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(350, 0, 480, 50);
    
    if (_isRecording) {
        pros::screen::set_pen(pros::c::COLOR_CYAN);
        pros::screen::fill_circle(460, 20, 15);
        pros::screen::set_pen(pros::c::COLOR_WHITE);
        pros::screen::print(pros::E_TEXT_SMALL, 355, 10, "REC-POS");
    } else if (_isPlaying) {
        pros::screen::set_pen(pros::c::COLOR_GREEN);
        pros::screen::fill_circle(460, 20, 15);
        pros::screen::set_pen(pros::c::COLOR_WHITE);
        pros::screen::print(pros::E_TEXT_SMALL, 360, 10, "PLAY");
    } else if (!recording.empty()) {
        pros::screen::set_pen(pros::c::COLOR_YELLOW);
        pros::screen::fill_circle(460, 20, 15);
        pros::screen::set_pen(pros::c::COLOR_WHITE);
        pros::screen::print(pros::E_TEXT_SMALL, 355, 10, "%d pts", recording.size());
    }
}
