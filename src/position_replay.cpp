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

size_t PositionReplay::findFrameIndexAtTime(uint64_t elapsedMicros) {
    if (recording.empty()) return 0;
    if (elapsedMicros >= recording.back().timestamp) 
        return recording.size() - 1;
    
    // Binary search
    size_t lo = 0, hi = recording.size() - 1;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if (recording[mid].timestamp < elapsedMicros) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
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
    
    // Removed task priority manipulation to avoid starvation risks
    
    master.print(0, 0, "RECORDING (POS)... ");
    master.rumble("-");
    
    drawStatusIndicator();
}

void PositionReplay::stopRecording(bool saveToSD) {
    _isRecording = false;
    
    // Removed task priority restoration
    
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
    
    // Only record at configured interval (Risk #1 fix: compare in microseconds for precision)
    if ((currentTime - lastRecordTime) / 1000 < recordingInterval) {
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
    
    // Safe push_back with hard limit
    if (recording.size() >= MAX_FRAMES) {
        master.print(0, 0, "MAX FRAMES REACHED!");
        stopRecording(true);
        return;
    }
    recording.push_back(frame);
    
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
    // Note: lastPlaybackButtons is now a class member, reset at playback start
    
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
    // Prevent starting playback while recording
    if (_isRecording) {
        master.print(0, 0, "STOP REC FIRST!    ");
        master.rumble("---");
        return;
    }
    
    if (recording.empty()) {
        if (!loadFromSD()) {
            master.print(0, 0, "NO RECORDING!      ");
            return;
        }
    }
    
    _isPlaying = true;
    _abortRequested = false;
    
    // Initial pneumatic states match physical defaults
    bool midScoring = true;   // Extended by default
    bool descore = false;     // Retracted by default
    bool unloader = false;    // Retracted by default
    
    // Reset pistons to known starting positions
    MidScoring.set_value(midScoring);
    Descore.set_value(descore);
    Unloader.set_value(unloader);
    
    // Reset pose to starting position
    chassis.setPose(0, 0, 0);
    pros::delay(50);
    
    master.print(0, 0, "REPLAYING (<>=STOP)");
    
    // Status indicator
    pros::screen::set_pen(pros::c::COLOR_GREEN);
    pros::screen::fill_circle(460, 20, 15);
    
    // Reset state for playback
    lastPlaybackButtons = 0;
    prevDistanceError = 0;
    prevHeadingError = 0;
    
    uint64_t startTime = pros::micros();
    // Use last frame's timestamp as total duration
    uint64_t totalDuration = recording.back().timestamp;
    
    // ===== TIME-SYNCED PURSUIT LOOP =====
    while (!_abortRequested && !checkEmergencyStop()) {
        uint64_t elapsed = pros::micros() - startTime;
        if (elapsed >= totalDuration) break;
        
        // Find target frame based on elapsed time
        size_t idx = findFrameIndexAtTime(elapsed);
        const WaypointFrame& target = recording[idx];
        
        // --- CUSTOM LIGHTWEIGHT PURE PURSUIT ---
        // We act like a pursuit controller following the moving target point
        
        lemlib::Pose current = chassis.getPose();
        float dx = target.x - current.x;
        float dy = target.y - current.y;
        float distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate desired heading toward target
        // atan2 returns radians, convert to degrees
        float targetHeading = std::atan2(dy, dx) * 180.0f / M_PI;
        
        // Heading error wrapping
        float headingError = targetHeading - current.theta;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        // PD controller using pre-tuned LemLib values
        // Lateral (forward): kP=4.25, kD=1.0
        // Angular (turn): kP=0.863, kD=0.235
        float kP_forward = 4.25f;
        float kD_forward = 1.0f;
        float kP_turn = 0.863f;
        float kD_turn = 0.235f;
        
        // Calculate derivative terms
        float distanceDerivative = distance - prevDistanceError;
        float headingDerivative = headingError - prevHeadingError;
        
        float forward = distance * kP_forward + distanceDerivative * kD_forward;
        float turn = headingError * kP_turn + headingDerivative * kD_turn;
        
        // Update previous errors for next iteration
        prevDistanceError = distance;
        prevHeadingError = headingError;
        
        // Clamp output
        if (forward > 127) forward = 127;
        if (forward < -127) forward = -127;
        if (turn > 127) turn = 127;
        if (turn < -127) turn = -127;
        
        // Special case: If we are extremely close to the point (within 0.5 inch),
        // match the recorded heading instead of driving to the point
        if (distance < 0.5f) {
            forward = 0;
            float thetaError = target.theta - current.theta;
            while (thetaError > 180) thetaError -= 360;
            while (thetaError < -180) thetaError += 360;
            
            // Use same PD values for heading correction
            float thetaDerivative = thetaError - prevHeadingError;
            turn = thetaError * kP_turn + thetaDerivative * kD_turn;
            prevHeadingError = thetaError;
            
            if (turn > 127) turn = 127;
            if (turn < -127) turn = -127;
        }
        
        // Apply drive power (Arcade: left = fwd + turn, right = fwd - turn)
        left_motors.move(forward + turn);
        right_motors.move(forward - turn);
        
        // --- APPLY MECHANISM STATES ---
        // Direct application from recorded frame
        Intake.move(target.intakePower);
        Outtake.move(target.outtakePower);
        
        // Pneumatic toggles with edge detection
        // Note: We use the helper logic inline here or call executeActions if preferred. 
        // Inline is clearer for this loop structure.
        
        if (wasPressed(target.buttons, lastPlaybackButtons, BTN_X)) {
            midScoring = !midScoring;
            MidScoring.set_value(midScoring);
        }
        if (wasPressed(target.buttons, lastPlaybackButtons, BTN_A)) {
            descore = !descore;
            Descore.set_value(descore);
        }
        if (wasPressed(target.buttons, lastPlaybackButtons, BTN_B)) {
            unloader = !unloader;
            Unloader.set_value(unloader);
        }
        
        lastPlaybackButtons = target.buttons;
        
        // Blink indicator
        uint32_t currentMs = pros::millis();
        if ((currentMs / 500) % 2 == 0) {
            pros::screen::set_pen(pros::c::COLOR_GREEN);
        } else {
            pros::screen::set_pen(pros::c::COLOR_DARK_GREEN);
        }
        pros::screen::fill_circle(460, 20, 15);
        
        pros::delay(20);
    }
    
    // Stop all motors
    left_motors.move(0);
    right_motors.move(0);
    Intake.move(0);
    Outtake.move(0);
    
    _isPlaying = false;
    _abortRequested = false;
    
    if (pros::competition::is_disabled()) {
         master.print(0, 0, "GAME DISABLED!     ");
    } else {
         master.print(0, 0, "REPLAY COMPLETE!   ");
    }
    
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
