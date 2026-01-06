# Kalman Filter Explained (No Scary Math!)

This guide explains how the Kalman Filter works without any Jacobian math or complicated matrix operations.

## The Big Idea

Imagine you're trying to figure out where your robot is, and you have two friends helping:

1. **Friend #1 (Odometry)**: Very fast updates, but makes small mistakes that add up over time
2. **Friend #2 (Vision)**: Very accurate, but only works sometimes (when AprilTags are visible)

The Kalman Filter is like a smart referee that listens to both friends and decides how much to trust each one.

## The Two Steps

### Step 1: PREDICT (using odometry)

**What happens:**
```
New position = Old position + How much we moved (from odometry)
```

**In code:**
```java
x = x + dx;  // Add the change in X
y = y + dy;  // Add the change in Y
heading = heading + dHeading;  // Add the change in heading
```

**Also:**
- We become LESS certain about our position (because odometry drifts)
- Uncertainty goes UP

**Example:**
- Robot thinks it's at (10, 5)
- Odometry says "you moved 2 inches forward"
- New estimate: (12, 5)
- Uncertainty increases from 1.0 to 1.1 inches

### Step 2: UPDATE (using vision)

**What happens:**
```
How much to trust vision = Our uncertainty / (Our uncertainty + Vision noise)
Correction = (Vision measurement - Our prediction) * Trust factor
New position = Old position + Correction
```

**In code:**
```java
// Calculate Kalman gain (trust factor)
K = P / (P + R);  // Between 0 and 1

// How different is vision from our prediction?
innovation = measuredX - x;

// Blend prediction and measurement
x = x + K * innovation;
```

**Also:**
- We become MORE certain (because vision confirmed/corrected us)
- Uncertainty goes DOWN

**Example:**
- We think we're at (12, 5) with uncertainty 1.1
- Vision says we're at (11.8, 5.2) with noise 0.5
- Kalman gain K = 1.1 / (1.1 + 0.5) = 0.69
- Innovation = 11.8 - 12 = -0.2
- Correction = 0.69 * (-0.2) = -0.14
- New estimate: 12 + (-0.14) = 11.86 inches
- Uncertainty decreases to 0.34 inches

## Understanding the Kalman Gain

The **Kalman Gain (K)** tells you how much to trust the measurement:

| K Value | Meaning | Why? |
|---------|---------|------|
| 0.0 - 0.3 | Trust odometry more | We're very certain OR vision is very noisy |
| 0.3 - 0.7 | Blend both | Balanced trust |
| 0.7 - 1.0 | Trust vision more | We're very uncertain OR vision is very accurate |

**Formula:**
```
K = Our Uncertainty / (Our Uncertainty + Vision Noise)
```

## The Magic: Self-Tuning

The beautiful thing about Kalman Filter is that **it automatically adjusts trust** based on circumstances:

**Scenario 1: Just started, high uncertainty**
```
P (uncertainty) = 2.0
R (vision noise) = 0.5
K = 2.0 / (2.0 + 0.5) = 0.8  ← Trust vision a lot!
```

**Scenario 2: Been seeing tags, low uncertainty**
```
P (uncertainty) = 0.3
R (vision noise) = 0.5
K = 0.3 / (0.3 + 0.5) = 0.38  ← Balanced trust
```

**Scenario 3: Haven't seen tags in a while, uncertainty growing**
```
P (uncertainty) = 3.0
R (vision noise) = 0.5
K = 3.0 / (3.0 + 0.5) = 0.86  ← Trust vision again!
```

## Tuning Parameters

You only need to tune 2 numbers per dimension:

### Q (Process Noise) - "How much does odometry drift?"
```java
KalmanFilter.Q_X = 0.1;  // Default
```

- **Too low (0.01)**: Filter trusts odometry too much, ignores vision
- **Too high (1.0)**: Filter jumps around, doesn't trust odometry
- **Good range**: 0.05 to 0.2

**How to find it:**
1. Disable vision: `FusedLocalizer.USE_VISION_CORRECTION = false`
2. Drive robot in a square
3. Measure drift after 30 seconds
4. Q ≈ (drift / 30) squared

### R (Measurement Noise) - "How accurate is vision?"
```java
KalmanFilter.R_X = 0.5;  // Default
```

- **Too low (0.1)**: Vision corrections are jumpy
- **Too high (2.0)**: Vision corrections are too slow
- **Good range**: 0.3 to 1.0

**How to find it:**
1. Place robot near tags
2. Watch position jump when tags detected
3. If jump is >2 inches, increase R
4. If corrections take >2 seconds, decrease R

## Debugging with Telemetry

The FusedLocalizer shows you everything:

```
Fused Estimate: (24.5, 36.2)     ← What the filter believes
Position Uncertainty: 0.8 in      ← How confident it is

Odometry: (24.7, 36.5)            ← What odometry says
Vision: (24.3, 36.0)              ← What vision says

Kalman Gain X: 0.62               ← How much trusting vision (0-1)
```

**Interpreting Kalman Gain:**
- Gain 0.8-1.0: "I'm lost, trust vision!"
- Gain 0.4-0.7: "Looks good, blend both"
- Gain 0.0-0.3: "I'm confident, mostly trust odometry"

## Common Scenarios

### Scenario: Robot jumps when it sees tags
**Problem:** Vision corrections are too aggressive
**Solution:** Increase R_X, R_Y (make filter trust vision less)
**Example:** Change R_X from 0.5 to 1.0

### Scenario: Vision corrections too slow
**Problem:** Filter trusts odometry too much
**Solution:** Increase Q_X, Q_Y OR decrease R_X, R_Y
**Example:** Change Q_X from 0.1 to 0.2

### Scenario: Filter ignores vision completely
**Problem:** Kalman gain too low
**Check:** Is vision actually providing data? Look at "Vision Active"
**Solution:** If vision is active but ignored, increase Q or decrease R

### Scenario: Position drifts even with tags visible
**Problem:** Kalman gain too low OR vision is wrong
**Debug:**
1. Check Kalman Gain - should be >0.3 when tags visible
2. Check Vision X/Y - does it match actual position?
3. Verify AprilTag positions in Limelight config

## Example Tuning Session

**Starting values:**
```java
Q_X = 0.1, Q_Y = 0.1, Q_HEADING = 0.05
R_X = 0.5, R_Y = 0.5, R_HEADING = 0.1
```

**Observation 1:** Robot jumps 3 inches when it sees tags
**Action:** Increase R to trust vision less
```java
R_X = 1.0, R_Y = 1.0  // Doubled
```

**Observation 2:** Corrections happen smoothly, but odometry still drifts
**Action:** Increase Q to trust odometry less
```java
Q_X = 0.2, Q_Y = 0.2  // Doubled
```

**Observation 3:** Perfect! Smooth corrections, minimal drift
**Final values:**
```java
Q_X = 0.2, Q_Y = 0.2, Q_HEADING = 0.05
R_X = 1.0, R_Y = 1.0, R_HEADING = 0.1
```

## Summary: The Formula (Simplified)

```
PREDICTION:
  position = position + odometry_delta
  uncertainty = uncertainty + Q

UPDATE (when vision available):
  trust_vision = uncertainty / (uncertainty + R)
  correction = (vision - position) * trust_vision
  position = position + correction
  uncertainty = uncertainty * (1 - trust_vision)
```

That's it! No matrices, no Jacobians, just simple addition and division.

## Quick Reference

| Want to... | Do this... |
|------------|-----------|
| Trust vision more | Decrease R OR increase Q |
| Trust vision less | Increase R OR decrease Q |
| Smoother corrections | Increase R |
| Faster corrections | Decrease R |
| Handle odometry drift | Increase Q |
| Trust odometry more | Decrease Q |

Remember: The filter **automatically adjusts** based on uncertainty. Your job is just to tell it roughly how noisy each sensor is!
