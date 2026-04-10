# Rover Control

**PID-driven autonomous rover navigation game written in C++**

A terminal-based game built around real control-systems and robotics engineering concepts. The rover can be driven manually or handed over to a PID autopilot that steers it through waypoints to the goal using heading-error and speed-error feedback loops — the same class of algorithm used in real autonomous systems.

Built as a personal project.

---

## Demo

```
##########################################################
#                                                        #
#  >        *            W                              #
#  ######################   ################            #
#                    *  *                               #
#                      *  ++                            #
#  #    ########## ########################################
#  #    ~~~~~~~~~~                          W            #
#  #    ~~~~~~~~~~                                      #
#       ~~~~~~~~~~                                      #
#  #    ~~~~~~~~~~                          #            #
#  ##############################  #########################
#                +++          W                      G  #
##########################################################
STATE: AUTOPILOT    SCORE:   312  TIME: 18.4s
ENERGY[||||||||||||||||    ] 81%  SPEED[========    ] 2.6
FWD: 4.2  WP:2/4  COLLISIONS:1  HDG:0.12rad
PID kp=2.5 ki=0.1 kd=0.8  integral=0.043
```

---

## Engineering Concepts

This project was designed to demonstrate core concepts from a control systems and robotics degree in a format that is immediately interactive and visible.

### PID Controller

Two independent PID controllers run every simulation step:

- **Steering PID** — measures the heading error (difference between desired angle toward the next waypoint and current rover heading), outputs an angular velocity correction
- **Throttle PID** — measures the speed error (difference between desired speed and current speed), outputs a linear acceleration

Both controllers implement **anti-windup** clamping on the integral term to prevent runaway accumulation when the rover is physically blocked.

```cpp
float update(float setpoint, float measured, float dt) {
    float err  = setpoint - measured;
    integral_ += err * dt;
    // Anti-windup: clamp integral contribution
    if (ki != 0.f)
        integral_ = myclamp(integral_, out_min/ki, out_max/ki);
    float deriv = (dt > 0.f) ? (err - prev_err_) / dt : 0.f;
    prev_err_ = err;
    return myclamp(kp*err + ki*integral_ + kd*deriv, out_min, out_max);
}
```

### Finite State Machine

The rover operates under a four-state FSM. Transitions are driven by energy level, player input, and proximity to waypoints.

```
 ┌──────────┐   p key    ┌───────────┐
 │  MANUAL  │ ─────────► │ AUTOPILOT │
 │          │ ◄───────── │           │
 └────┬─────┘   p key    └─────┬─────┘
      │                        │
   energy=0                energy=0
      │                        │
      ▼                        ▼
 ┌────────────┐          ┌─────────────┐
 │ RECHARGING │          │ GOAL REACHED│
 └────────────┘          └─────────────┘
```

### Sensor Simulation (DDA Raycasting)

Three proximity sensors are simulated each tick using a DDA (Digital Differential Analyzer) raycast — the same algorithm at the heart of classic raycasting renderers like Wolfenstein 3D.

```cpp
float raycast(const Map& map, float angle, float maxd) const {
    float dx = std::cos(angle), dy = std::sin(angle);
    for (float d = 0.15f; d < maxd; d += 0.15f)
        if (map.solid((int)(pos.y + dy*d), (int)(pos.x + dx*d)))
            return d;
    return maxd;
}
```

Sensors fire forward, 90° left, and 90° right. The autopilot uses the forward sensor to reduce target speed when a wall is approaching.

### Physics Engine

Each simulation tick runs 12 sub-steps at 0.06s each, giving smooth physics from a single player input:

- Continuous position updated from heading vector and speed scalar
- Tile-based collision with bounce response (`speed *= -0.3`)
- Terrain effects: swamp tiles multiply energy drain by 2.5× and apply speed damping; boost pads add velocity and reduce drain
- Energy depletion triggers the RECHARGING state, halting movement until battery recovers
- Friction applied each step in manual mode (`speed *= 0.95`)

---

## How to Run

### OnlineGDB (no installation needed)

1. Go to [onlinegdb.com](https://onlinegdb.com)
2. Set the language to **C++**
3. Paste the contents of `rover_control.cpp`
4. Click **Run**

### Local (Linux / macOS)

```bash
g++ -std=c++11 -o rover rover_control.cpp
./rover
```

### Local (Windows)

```bash
g++ -std=c++11 -o rover.exe rover_control.cpp
rover.exe
```

---

## Controls

The game uses turn-based input — type a command and press **Enter**. The physics engine runs 12 simulation sub-steps per command so the rover moves a meaningful distance each turn.

| Key | Action |
|-----|--------|
| `w` | Accelerate forward |
| `s` | Reverse / brake |
| `a` | Turn left |
| `d` | Turn right |
| `q` | Emergency brake |
| `p` | Toggle PID autopilot on/off |
| `.` or Enter | Coast (advance simulation, no input) |
| `r` | Restart |
| `x` | Quit |

**Tip:** Type `p` then keep pressing Enter to watch the PID autopilot navigate the rover autonomously through all waypoints.

---

## Map Legend

| Symbol | Meaning |
|--------|---------|
| `#` | Wall (impassable) |
| `~` | Swamp — slows rover, drains energy 2.5× faster |
| `+` | Boost pad — increases speed, reduces energy drain |
| `W` | Waypoint — collect all for points |
| `G` | Goal — reach this to complete the mission |
| `*` | Rover trail (last 20 positions) |
| `> v < ^` | Rover with heading arrow |

---

## Scoring

| Event | Points |
|-------|--------|
| Each waypoint reached | +50 |
| Reaching the goal | +500 |
| Movement (per step) | +speed × 0.3 |

Minimise collisions and avoid the swamp to maximise your score.

---

## Project Structure

```
rover-control/
├── rover_control.cpp   # Full source — single file, no dependencies
├── README.md
└── LICENSE
```

---

## Skills Demonstrated

- **C++11** — classes, templates, lambdas, STL containers, chrono
- **Control Systems** — PID controller with anti-windup, dual-loop (heading + speed)
- **Robotics** — sensor simulation, state estimation, waypoint navigation
- **Game Engineering** — FSM, tile-based physics, collision detection, raycasting
- **Software Design** — separation of simulation, input, and rendering layers

---

## License

MIT — see `LICENSE` for details.
