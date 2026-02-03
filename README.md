# VEX V5 Position-Based Auton Recorder

**Record driver movements → Replay with LemLib's self-correcting motion control**

Uses LemLib's odometry to record (X, Y, Heading) positions instead of raw motor values, resulting in more robust and accurate autonomous playback.

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
| **Position Recording** | Captures (X, Y, θ) at 20 samples/sec |
| **Self-Correcting** | LemLib's closed-loop motion control |
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
positionReplay.setRecordingInterval(25);       // Even faster: 40 samples/sec
positionReplay.setCountdownDuration(5000);     // 5 second countdown
positionReplay.setActionTriggerRadius(5.0f);   // Trigger radius in inches
```

---

## 📊 Technical Specs

| Spec | Value |
|------|-------|
| Sample Rate | 40 Hz (25ms intervals) |
| File Location | `/usd/position_recording.bin` |
| Max Recording | ~5+ minutes |
| Data Per Frame | X, Y, θ, motors, buttons, timestamp |
| Playback Method | `chassis.moveToPose()` |

---

## 🔧 How It Works

### Recording
```
1. Resets odometry to (0, 0, 0)
2. Every 50ms: captures chassis.getPose()
3. Records motor powers & button states
4. Marks frames with mechanism actions
```

### Playback (Hybrid Mode)
```
For each waypoint:
  → If distance > 1": moveToPose(x, y, θ)
  → If only heading differs: turnToHeading(θ)  
  → If action frame: execute motors/pneumatics
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
