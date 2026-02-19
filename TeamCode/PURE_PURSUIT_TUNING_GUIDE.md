# Pure Pursuit Tuning Quick Reference

## Using the Tuner OpMode

1. **Deploy the robot** and connect to FTC Dashboard
2. **Select "Pure Pursuit Tuner"** from TeleOp
3. **Open FTC Dashboard** at http://192.168.43.1:8080/dash
4. Follow the on-screen instructions for each stage

## Controls

- **A** = Run test path
- **B** = Next tuning stage
- **X** = Previous tuning stage
- **Y** = Reset robot position to (0,0,0)

## Tuning Parameters (in FTC Dashboard)

### SixWheelPID Class
- `pXY` - Linear P gain (response speed)
- `dXY` - Linear D gain (damping)
- `pR` - Rotation P gain (turn speed)
- `dR` - Rotation D gain (turn damping)
- `STOP_DISTANCE` - Distance to stop linear movement
- `BACKWARDS_THRESHOLD` - Angle to switch to backwards (degrees)
- `FORWARDS_THRESHOLD` - Angle to switch to forwards (degrees)

### SixWheelDrive Class
- `LOOK_AHEAD_DIST` - How far ahead robot looks on path
- `END_TOLERANCE` - Distance to consider path complete
- `HEADING_TOLERANCE` - Angle to consider turn complete (degrees)

## Stage-by-Stage Guide

### Stage 1: Linear P Gain
**Set:** `dXY = 0.0` (disable D)
**Tune:** `pXY`
**Watch:** Linear velocity should be smooth

- **Too slow/sluggish?** → Increase `pXY` (try 0.15-0.2)
- **Too aggressive/overshoots?** → Decrease `pXY` (try 0.05-0.07)
- **Target:** Reasonable speed, no wild oscillations

### Stage 2: Rotation P Gain
**Set:** `dR = 0.0` (disable D)
**Tune:** `pR`
**Watch:** Rotation should be smooth, not spinning

- **Too slow to turn?** → Increase `pR` (try 0.04-0.05)
- **Spins/overshoots?** → Decrease `pR` (try 0.015-0.02)
- **Target:** Smooth turns without spinning

### Stage 3: Linear D Gain
**Set:** `dXY = pXY * 0.5` (starting point)
**Tune:** `dXY`
**Watch:** Motion should be smooth, no vibration

- **Oscillates back/forth?** → Increase `dXY` (+0.02 at a time)
- **Jerky/jittery/vibrating?** → Decrease `dXY` (-0.01 at a time)
- **Sluggish response?** → Decrease `dXY`
- **Target:** Smooth motion, no oscillation

### Stage 4: Rotation D Gain
**Set:** `dR = pR * 0.5` (starting point)
**Tune:** `dR`
**Watch:** Rotation should be smooth

- **Wobbles side-to-side?** → Increase `dR`
- **Jerky rotation?** → Decrease `dR`
- **Target:** Smooth, controlled rotation

### Stage 5: Look-Ahead Distance
**Tune:** `LOOK_AHEAD_DIST`
**Watch:** Smoothness at waypoints

- **Jerky at waypoints?** → Increase (try 10-15)
- **Cuts corners too much?** → Decrease (try 6-8)
- **Typical range:** 8-15 inches
- **Target:** Fluid path following through waypoints

### Stage 6: Final Validation
**Test:** Run all test paths multiple times

Check for:
- ✓ Smooth motion (no jerking)
- ✓ No oscillations
- ✓ Reaches target accurately
- ✓ Good speed throughout
- ✓ Smooth through waypoints

## Common Issues & Solutions

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Jerky at waypoints | Look-ahead too small | Increase `LOOK_AHEAD_DIST` to 10-15 |
| Oscillates back/forth | P too high or D too low | Decrease `pXY` by 30% OR increase `dXY` by 50% |
| Jittery/vibrating | D too high (noisy velocity) | Decrease `dXY` by 50% |
| Slow to respond | P too low or D too high | Increase `pXY` by 50% OR decrease `dXY` by 30% |
| Stops short of target | STOP_DISTANCE too large | Decrease `STOP_DISTANCE` to 1.5 or 2.0 |
| Violent rocking | Backwards driving toggling | Increase hysteresis gap (BACKWARDS_THRESHOLD - FORWARDS_THRESHOLD) |

## Typical Starting Values

```java
// Conservative (smooth but slow)
pXY = 0.08, dXY = 0.04
pR = 0.025, dR = 0.008
LOOK_AHEAD_DIST = 10.0

// Moderate (balanced)
pXY = 0.1, dXY = 0.05
pR = 0.03, dR = 0.01
LOOK_AHEAD_DIST = 12.0

// Aggressive (fast but may be jerky)
pXY = 0.15, dXY = 0.06
pR = 0.04, dR = 0.012
LOOK_AHEAD_DIST = 8.0
```

## Recording Your Final Gains

Once tuned, **write down your gains** and update the default values in code:

1. Open `SixWheelPID.java`
2. Update lines with your tuned values:
   ```java
   public static double pXY = YOUR_VALUE, dXY = YOUR_VALUE;
   public static double pR = YOUR_VALUE, dR = YOUR_VALUE;
   ```

3. Open `SixWheelDrive.java`
4. Update:
   ```java
   public static double LOOK_AHEAD_DIST = YOUR_VALUE;
   ```

## Pro Tips

1. **Tune one parameter at a time** - Don't change multiple gains simultaneously
2. **Use small increments** - Especially for D gains (±0.01)
3. **Test multiple paths** - Don't just test one straight line
4. **Watch telemetry** - The numbers tell the story
5. **Record everything** - Keep notes of what values work
6. **Field conditions matter** - Battery level, floor surface affect behavior
7. **Re-tune if needed** - Different robots/conditions may need different gains

## Troubleshooting Dashboard

If FTC Dashboard isn't showing your variables:
1. Ensure `@Config` annotation is on the class
2. Variables must be `public static`
3. Refresh the dashboard page
4. Check you're connected to the robot's WiFi

## Need Help?

If stuck on a particular stage:
- Use **X** button to go back and re-tune previous stage
- Try the "Conservative" starting values above
- Check "Common Issues" table
- Verify your robot's mechanical setup (loose wheels, friction, etc.)

---

**Remember:** Perfect is the enemy of good. Your goal is "smooth enough" not "perfect."
Once the robot follows paths reasonably well, you're done!
