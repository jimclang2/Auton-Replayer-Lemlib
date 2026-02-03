#pragma once
#include "main.h"
#include <vector>
#include <string>

/**
 * Position-Based Autonomous Recording System
 * 
 * Records robot position (X, Y, Heading) using LemLib's odometry system
 * and replays using pure pursuit with mechanism action pauses.
 */

// Single waypoint frame - captures position and mechanism states at a moment in time
struct WaypointFrame {
    // Position data from chassis.getPose()
    float x;            // X position in inches
    float y;            // Y position in inches  
    float theta;        // Heading in degrees
    
    // Timing
    uint64_t timestamp; // Time since recording started (microseconds)
    
    // Mechanism states
    int8_t intakePower;     // Intake motor power (-127 to 127)
    int8_t outtakePower;    // Outtake motor power (-127 to 127)
    
    // Button states packed into bitflags for pneumatics
    // Bit 0: R1 (intake forward) - reference only
    // Bit 1: R2 (intake reverse) - reference only  
    // Bit 2: L1 (outtake forward) - reference only
    // Bit 3: L2 (outtake reverse) - reference only
    // Bit 4: X (mid-scoring toggle)
    // Bit 5: A (descore toggle)
    // Bit 6: B (unloader toggle)
    uint8_t buttons;
    
    // Flag to indicate if mechanism action occurred at this waypoint
    bool hasAction;     // True if any mechanism was activated at this frame
};

// Button bit positions (same as old system for compatibility)
constexpr uint8_t BTN_R1 = 0;
constexpr uint8_t BTN_R2 = 1;
constexpr uint8_t BTN_L1 = 2;
constexpr uint8_t BTN_L2 = 3;
constexpr uint8_t BTN_X  = 4;
constexpr uint8_t BTN_A  = 5;
constexpr uint8_t BTN_B  = 6;

/**
 * Position-based recording and playback system using LemLib odometry
 */
class PositionReplay {
private:
    std::vector<WaypointFrame> recording;
    uint64_t recordStartTime = 0;
    bool _isRecording = false;
    bool _isPlaying = false;
    bool _abortRequested = false;
    
    // Previous button states for edge detection
    uint8_t prevButtons = 0;
    
    // Configuration
    uint32_t recordingInterval = 25;        // Recording interval in ms (25ms = 40 samples/sec)
    uint32_t countdownDuration = 3000;      // Countdown before recording (ms)
    float actionTriggerRadius = 3.0f;       // Inches - radius for position-based action triggering
    float lookaheadDistance = 15.0f;        // Pure pursuit lookahead distance in inches
    
    // File path for SD card storage
    std::string filePath = "/usd/position_recording.bin";
    
    // Last record time for interval-based recording
    uint64_t lastRecordTime = 0;
    
    // Helper methods
    void displayCountdown(int secondsRemaining);
    bool checkEmergencyStop();
    uint8_t packButtons();
    bool wasPressed(uint8_t current, uint8_t prev, uint8_t bit);
    void executeActions(const WaypointFrame& frame, bool& midScoring, bool& descore, bool& unloader);
    
public:
    // ==================== Recording ====================
    
    /**
     * Start recording with countdown
     * Resets odometry to (0, 0, 0) for consistent reference
     */
    void startRecording();
    
    /**
     * Stop recording and optionally save to SD
     */
    void stopRecording(bool saveToSD = true);
    
    /**
     * Record current position if interval has elapsed
     * Call this every loop iteration (~20ms) - it internally handles timing
     */
    void recordFrame();
    
    // ==================== Playback ====================
    
    /**
     * Hybrid playback: Pure pursuit for motion, pauses for mechanism actions
     * Uses chassis.moveToPose() for smooth motion between waypoints
     */
    void playback();
    
    /**
     * Abort playback (emergency stop)
     */
    void abortPlayback();
    
    // ==================== Data Management ====================
    
    /**
     * Clear the current recording
     */
    void clearRecording();
    
    /**
     * Save recording to SD card in binary format
     */
    bool saveToSD();
    
    /**
     * Load recording from SD card
     */
    bool loadFromSD();
    
    // ==================== Getters/Setters ====================
    
    int getFrameCount() const { return recording.size(); }
    uint32_t getDuration() const;
    bool isRecording() const { return _isRecording; }
    bool isPlaying() const { return _isPlaying; }
    
    void setRecordingInterval(uint32_t ms) { recordingInterval = ms; }
    void setCountdownDuration(uint32_t ms) { countdownDuration = ms; }
    void setActionTriggerRadius(float inches) { actionTriggerRadius = inches; }
    void setLookaheadDistance(float inches) { lookaheadDistance = inches; }
    void setFilePath(const std::string& path) { filePath = path; }
    
    // ==================== Status Display ====================
    
    bool isSDCardInserted() const;
    void drawStatusIndicator();
};

// Global instance
extern PositionReplay positionReplay;
