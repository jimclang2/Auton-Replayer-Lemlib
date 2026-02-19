# VEX V5 Position-Based Auton Recorder

**Record driver movements → Replay with time-synced Pure Pursuit**

Uses LemLib's odometry to record (X, Y, Heading) positions and replays them with a custom pursuit controller for smooth, accurate autonomous playback.

---

## ⚙️ PID Configuration

The playback uses a **PD controller** (Proportional + Derivative) for smooth path following.

📍 **Location:** `src/position_replay.cpp` → inside `playback()`

```cpp
// Match these to your LemLib lateral/angular PID settings
float kP_forward = 5.0f;    // Lateral kP
float kD_forward = 8.0f;    // Lateral kD
float kP_turn = 1.7f;       // Angular kP
float kD_turn = 14.0f;      // Angular kD
```

💡 **Tip:** Use the same values from your LemLib `lateral_controller` and `angular_controller` for consistent behavior.

| Symptom | Fix |
|---------|-----|
| Robot overshoots/oscillates | Increase kD values |
| Robot drifts off path | Increase `kP_forward` |
| Robot slow to turn | Increase `kP_turn` |
| Too aggressive/jerky | Lower kP values |

---

## 🚀 Quick Start

### Recording
| Method | Action |
|--------|--------|
| **Screen** | Tap `RECORD` → Drive → Tap `STOP` |
| **Controller** | `UP` → Drive → `DOWN` |

### Playback  
| Method | Action |
|--------|--------|
| **Screen** | Tap `PLAY` |
| **Controller** | `LEFT` |

### Emergency Stop
Press `UP + DOWN` arrows simultaneously during playback.

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| **Position Recording** | Captures (X, Y, θ) at 40 samples/sec |
| **Time-Synced Playback** | Matches recording timing exactly |
| **Custom Pure Pursuit** | Smooth path following with PD controller |
| **Mechanism Actions** | Records intake, outtake, pneumatics |
| **SD Card Storage** | Recordings persist across power cycles |
| **Compact Files** | ~6KB per minute of recording |

---

## 🎮 Controller Buttons

| Button | Function |
|--------|----------|
| `UP` | Start recording |
| `DOWN` | Stop recording (auto-saves) |
| `LEFT` | Test playback |
| `UP + DOWN` | Emergency stop |

---

## 💡 Tips for Best Results

1. **Same starting position** - Place robot identically each time
2. **Wait for IMU** - Let robot sit 2-3 seconds on startup
3. **Smooth driving** - Consistent movements replay best
4. **Check odometry** - Ensure tracking wheel works correctly
5. **Test first** - Always verify before competition

---

## ⚙️ Configuration

```cpp
// Customize in your code:
positionReplay.setRecordingInterval(25);       // 40 samples/sec (default)
positionReplay.setCountdownDuration(5000);     // 5 second countdown
positionReplay.setActionTriggerRadius(5.0f);   // Trigger radius in inches
```

---

## 📊 Technical Specs

| Spec | Value |
|------|-------|
| Sample Rate | 40 Hz (25ms intervals) |
| File Location | `/usd/position_recording.bin` |
| Max Recording | ~2 minutes (5000 frames) |
| Data Per Frame | X, Y, θ, motors, buttons, timestamp |
| Playback Method | Time-synced PD controller pursuit |

---

## 🔧 How It Works

### Recording
```
1. Resets odometry to (0, 0, 0)
2. Every 25ms: captures chassis.getPose()
3. Records motor powers & button states
4. Saves to SD card on stop
```

### Playback (Time-Synced Pursuit)
```
Start timer
Loop every 20ms:
  → Find frame matching elapsed time (binary search)
  → Calculate distance/heading error to target
  → Apply PD controller: motors = error × kP + Δerror × kD
  → Apply intake/outtake/pneumatics from frame
```

---

## 📁 Project Structure

```
src/
├── position_replay.cpp   ← Recording & playback logic
├── main.cpp              ← UI and control loop
└── ...

include/
├── position_replay.h     ← WaypointFrame struct & class
└── ...
```

---

## 📜 License

Apache-2.0
