# VEX V5 Position-Based Auton Recorder

**Record driver movements → Replay with time-synced Pure Pursuit**

Uses LemLib's odometry to record (X, Y, Heading) positions and replays them with a custom pursuit controller for smooth, accurate autonomous playback.

---

## ⚠️ IMPORTANT: Tune PID Before Use

The playback uses a P controller for path following. **You must tune these values for your robot:**

📍 **Location:** `src/position_replay.cpp` → Lines 316-317 (inside `playback()`)

```cpp
float kP_forward = 8.0f;   // Higher = more aggressive pursuit (try 5-15)
float kP_turn = 2.0f;      // Higher = faster heading correction (try 1-5)
```

| Symptom | Fix |
|---------|-----|
| Robot overshoots/oscillates | Lower both values |
| Robot drifts off path | Increase `kP_forward` |
| Robot slow to turn | Increase `kP_turn` |
| Robot too aggressive | Lower both values by 50% |

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
Press `LEFT + RIGHT` arrows simultaneously during playback.

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| **Position Recording** | Captures (X, Y, θ) at 40 samples/sec |
| **Time-Synced Playback** | Matches recording timing exactly |
| **Custom Pure Pursuit** | Smooth path following with P controller |
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
| `LEFT + RIGHT` | Emergency stop |

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
| Playback Method | Time-synced P controller pursuit |

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
  → Apply P controller: motors = error × kP
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
