# ---------------------------------------------------------------------------- #
#                                                                              #
#   Module:       main.py                                                      #
#   Author:       Margaret Liu                                                 #
#   Created:      1/13/2025, 10:24:50 PM                                       #
#   Description:  Tank drivetrain with PID autonomous + sensor validation      #
#                                                                              #
# ---------------------------------------------------------------------------- #

import time
import math
from vex import *

# ============================================================
# ROBOT CONFIGURATION
# ============================================================

brain      = Brain()
controller = Controller()

# Half-track widths (center of left wheels to robot centerline,
# and center of right wheels to robot centerline), in inches.
# Used in the heading calculation: rotation = (left_arc - right_arc) / (tl + tr)
tl = 6.75   # left half-track width
tr = 6.75   # right half-track width

pi = 3.14159

# Converts motor shaft rotations (turns) to linear inches at the wheel.
# Formula: pi * wheel_diameter / gear_ratio
# Wheel diameter = 3.25 in, gear ratio = 36/24 = 1.5  →  pi * 3.25 / 1.5
wheelFactor = pi * 3.25 / (5 / 3)

# ---- Drive motors (tank drive: 2 left, 2 right) ----
# reversed=True  means the motor's positive direction spins the wheel forward
motorFL = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)   # front-left
motorFR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)  # front-right
motorBL = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)   # back-left
motorBR = Motor(Ports.PORT3,  GearSetting.RATIO_6_1, False)  # back-right

# ---- Sensors ----
inertialSensor    = Inertial(Ports.PORT7)
trackingWheelVertL  = Rotation(Ports.PORT4, False)  # vertical tracking wheel  (forward/back)
trackingWheelHorizR = Rotation(Ports.PORT5, True)  # horizontal tracking wheel (strafe)
distVert  = Distance(Ports.PORT18)  # vertical-facing distance sensor
distHoriz = Distance(Ports.PORT19)   # horizontal-facing distance sensor

toggleFlipper = Motor(Ports.PORT6, GearSetting.RATIO_36_1, False)  # example motor for a toggle mechanism
# ---- Claw ----
digital_out_a = DigitalOut(brain.three_wire_port.a)

# ---- Controllers ----
controller_1 = Controller(PRIMARY)

# ---- AI Vision (used for game element detection) ----
class GameElements:
    MOBILE_GOAL = 0
    RED_RING    = 1
    BLUE_RING   = 2
    SKIP        = -1

AI_clamp = AiVision(Ports.PORT15, AiVision.ALL_AIOBJS)


def claw_open():
    """Release the claw. TODO: wire to whichever mechanism is finalized
    (pneumatic solenoid via digital_out_a, or a claw motor)."""
    pass


def claw_close():
    """Clamp the claw onto whatever ring is currently between the jaws.
    TODO: wire to whichever mechanism is finalized
    (pneumatic solenoid via digital_out_a, or a claw motor)."""
    pass


# ============================================================
# MOTOR HELPERS
# ============================================================

def motor_Stop():
    """Coast all four drive motors to a stop."""
    motorFL.stop(); motorFR.stop()
    motorBL.stop(); motorBR.stop()

def motor_hold():
    """Lock all four drive motors in place (active braking)."""
    motorFL.stop(HOLD); motorFR.stop(HOLD)
    motorBL.stop(HOLD); motorBR.stop(HOLD)

def motor_brake():
    """Apply friction braking to all four drive motors."""
    motorFL.stop(BRAKE); motorFR.stop(BRAKE)
    motorBL.stop(BRAKE); motorBR.stop(BRAKE)

def motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed):
    """Spin all four drive motors at the given velocity percentages.
    Positive values = forward for each motor (reversed flag handled at init).
    """
    motorFL.spin(DirectionType.FORWARD, motorFLSpeed, VelocityUnits.PERCENT)
    motorFR.spin(DirectionType.FORWARD, motorFRSpeed, VelocityUnits.PERCENT)
    motorBL.spin(DirectionType.FORWARD, motorBLSpeed, VelocityUnits.PERCENT)
    motorBR.spin(DirectionType.FORWARD, motorBRSpeed, VelocityUnits.PERCENT)


# ============================================================
# SENSOR HELPERS
# ============================================================

def Reset_all():
    """Zero all drive motor encoder positions.
    Called at the start of every autonomous move so distance is measured
    from the beginning of that move only.
    The IMU is NOT reset here — it accumulates heading across all moves.
    """
    motorFL.set_position(0, DEGREES)
    motorFR.set_position(0, DEGREES)
    motorBL.set_position(0, DEGREES)
    motorBR.set_position(0, DEGREES)

def get_Rotation_Sensor_Position():
    """Return (left_turns, right_turns) from the two representative drive motors.
    motorFL is the left-side reference; motorBR is the right-side reference.
    Multiply by wheelFactor to convert turns → inches.
    """
    return motorFL.position(TURNS), motorBR.position(TURNS)

def calculate_Rotation_From_Wheels(xLeft, yRight, wheelFactor, tl, tr):
    """Convert left/right encoder turns into a heading change (radians).
    Uses the differential drive arc formula:
        rotation = (left_arc - right_arc) / track_width
    where track_width = tl + tr (total width between contact points).
    """
    return (xLeft * wheelFactor - yRight * wheelFactor) / (tl + tr)

def reset_tracking_wheels():
    """Zero both dedicated tracking wheel encoders.
    Call this before each move when you need a clean distance measurement
    from the tracking wheels (they are not reset by Reset_all).
    """
    trackingWheelVertL.set_position(0, TURNS)
    trackingWheelHorizR.set_position(0, TURNS)

def get_Tracking_Wheel_Position():
    """Return (vertical_turns, horizontal_turns) from the tracking wheels.
    Vertical wheel measures forward/backward travel.
    Horizontal wheel measures sideways (strafe) travel.
    Multiply by ODO_FACTOR to convert turns → inches.
    """
    return trackingWheelVertL.position(TURNS), trackingWheelHorizR.position(TURNS)


# ============================================================
# AUTONOMOUS PID — ENCODER-DRIVEN
# ============================================================

def autonomousPID(
    target,
    initialMaxSpeedLimit,
    maxSpeedLimit,
    time_out,
    export_flag,
    Kp_l,
    Kp_r,
    previousError=None,
    exit_velocity_pct=0.0,
    slow_down_distance=6.0,
    settle_error_dist=0.5,
    settle_error_heading=0.05,
    settle_loops=8,
    stop_at_end=True,
    heading_guard_deg=2.0,
    heading_guard_floor=0.0,
):
    """Drive a straight line or in-place turn using PID feedback.

    Distance feedback : average of motorFL and motorBR encoder positions.
    Heading feedback  : VEX Inertial Sensor (IMU), in radians.

    Parameters
    ----------
    target              : [distance_in, heading_rad] — where to go.
    initialMaxSpeedLimit: speed cap for the first 100 loop iterations (ramp-up).
    maxSpeedLimit       : normal speed cap after ramp-up.
    time_out            : loop iteration limit before the move is force-stopped.
    export_flag         : set to 1 to print debug telemetry every 10 loops.
    Kp_l                : proportional gain for distance (linear) error.
    Kp_r                : proportional gain for heading (rotation) error.
    exit_velocity_pct   : if non-zero and stop_at_end=False, the robot keeps
                          moving at this speed after settling (chain moves).
    slow_down_distance  : start blending speed toward exit_velocity_pct within
                          this many inches of the target (soft deceleration).
    settle_error_dist   : distance window (in) to count as "settled".
    settle_error_heading: heading window (rad) to count as "settled".
    settle_loops        : consecutive loops inside the settle window required
                          before the move is declared complete.
    stop_at_end         : if False, motors stay spinning at exit_velocity_pct.
    heading_guard_deg   : if heading error exceeds this angle, forward output is
                          scaled down so the robot corrects heading before driving.
    heading_guard_floor : minimum forward scaling factor when guard is active.
    """

    global hook_flag, combine
    combine=[]
    initialVertDist = distVert.object_distance(INCHES)
    initialHorizDist = distHoriz.object_distance(INCHES)
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    Reset_all()   # zero encoders at the start of each move

    if previousError is None:
        previousError = [0.0, 0.0]

    # PID gains
    Kp = Kp_l       # linear proportional
    Ki = 0.0         # linear integral (disabled)
    KpRotation = Kp_r  # heading proportional
    KiRotation = 0.0   # heading integral (disabled)
    # Derivative gains — increase these if you see overshoot or oscillation
    Kd = 0.0
    KdRotation = 0.0

    # currentPosition tracks how far we've gone and what heading we're at
    currentPosition = [0.0, inertialSensor.rotation() * pi / 180.0]

    error          = [0.0, 0.0]  # [distance_error_in, heading_error_rad]
    integral       = 0.0         # accumulated distance error (for I term)
    headingIntegral = 0.0        # accumulated heading error  (for I term)

    dt            = 0.005   # loop period: 5 ms → 200 Hz
    counter       = 0
    settle_counter = 0
    leftRotation   = 0.0
    rightRotation  = 0.0

    # heading_guard_deg is copied to a local name for clarity
    HEADING_GUARD_DEG = heading_guard_deg

    while True:
        # 1. Compute how far off we are from the target
        error[0] = target[0] - currentPosition[0]   # inches remaining
        error[1] = target[1] - currentPosition[1]   # heading error (rad)

        # 2. Accumulate integrals and estimate derivatives
        integral        += error[0] * dt
        headingIntegral += error[1] * dt
        derivative        = (error[0] - previousError[0]) / dt
        headingDerivative = (error[1] - previousError[1]) / dt

        # 3. Calculate raw PID outputs
        xOutput   = (Kp * error[0]) + (Ki * integral) + (Kd * derivative)
        turnSpeed = (KpRotation * error[1]
                     + KiRotation * headingIntegral
                     + KdRotation * headingDerivative)

        # 4. Heading guard: if heading error is large, reduce forward output
        #    so the robot straightens out before accelerating forward.
        heading_error_deg = error[1] * 180.0 / pi
        abs_head_err = abs(heading_error_deg)
        if HEADING_GUARD_DEG > 0 and abs_head_err > HEADING_GUARD_DEG:
            scale = HEADING_GUARD_DEG / abs_head_err
            scale = max(heading_guard_floor, min(1.0, scale))
            xOutput *= scale

        # 5. Mix forward and turn components into individual motor speeds
        motorFLSpeed = xOutput + turnSpeed
        motorFRSpeed = xOutput - turnSpeed
        motorBLSpeed = xOutput + turnSpeed
        motorBRSpeed = xOutput - turnSpeed

        # 6. Speed limiting
        #    First 100 loops: use initialMaxSpeedLimit as a gentle ramp-up cap.
        #    After that: blend toward exit_velocity_pct near the target (soft stop).
        distance_remaining = abs(error[0])
        exit_speed    = abs(exit_velocity_pct)
        dynamic_limit = maxSpeedLimit
        # Skip distance-based slow-down for pure rotations: when target[0]==0,
        # distance_remaining stays ~0 the whole move, which collapses dynamic_limit
        # to 0 after ramp-up and stalls the turn before it reaches the target.
        if slow_down_distance > 0 and abs(target[0]) > 0.001:
            blend = min(1.0, max(0.0, distance_remaining / slow_down_distance))
            dynamic_limit = exit_speed + (maxSpeedLimit - exit_speed) * blend

        if counter <= 100:
            cap = initialMaxSpeedLimit
        else:
            cap = dynamic_limit

        maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                       abs(motorBLSpeed), abs(motorBRSpeed),
                       cap)
        if maxSpeed > cap:
            ratio = cap / maxSpeed
            motorFLSpeed *= ratio; motorFRSpeed *= ratio
            motorBLSpeed *= ratio; motorBRSpeed *= ratio

        # 7. Send speeds to the motors
        motor_Motion(motorFLSpeed, motorFRSpeed,
                     motorBLSpeed, motorBRSpeed)

        # 8. Store errors for the next loop's derivative calculation
        previousError[0] = error[0]
        previousError[1] = error[1]

        # 9. Update our estimated position from encoder average + IMU heading
        leftRotation, rightRotation = get_Rotation_Sensor_Position()
        if leftRotation == rightRotation:
            currentPosition[0] = leftRotation * wheelFactor
        else:
            currentPosition[0] = ((leftRotation + rightRotation) / 2.0) * wheelFactor
        currentPosition[1] = inertialSensor.rotation() * pi / 180.0
        # Uncomment the line below to use wheel-differential heading instead of IMU:
        # currentPosition[1] = calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr)

        # 10. Optional debug print every 10 loops
        if counter % 6 == 0 and export_flag == 1:

            # print(counter, end="\t")
            # print('{:.5f}'.format(error[0]), end="\t")
            # print('{:.5f}'.format(error[1]), end="\t")
            # print('{:.5f}'.format(xOutput), end="\t")
            # print('{:.5f}'.format(turnSpeed), end="\t")
            # print('{:.5f}'.format(motorFLSpeed), end="\n")

            leftRotation, rightRotation = get_Rotation_Sensor_Position()
            encoderPos = ((leftRotation + rightRotation) / 2.0) * wheelFactor
            enc_heading_deg = calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr) * 180.0 / math.pi
            data_export = [counter, encoderPos, error[0], derivative,
                    trackingWheelVertL.position(TURNS)*2*math.pi,
                    trackingWheelHorizR.position(TURNS)*2*math.pi,
                    distVert.object_distance(INCHES)-initialVertDist, distHoriz.object_distance(INCHES)-initialHorizDist,
                    currentPosition[1]*180/math.pi, error[1]*180/math.pi, headingDerivative,
                    calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr) * 180.0 / math.pi]
            # data_export = [counter, error[0], derivative, error[1]*180, headingDerivative,
            #             encoderPos, trackingWheelVert.position(TURNS)*2 * math.pi, trackingWheelHoriz.position(TURNS)*2*math.pi]
            combine += [data_export]


        # 11. Settle check — both errors must stay inside their windows for
        #     settle_loops consecutive iterations before the move is done.
        if abs(error[0]) < settle_error_dist and abs(error[1]) < settle_error_heading:
            settle_counter += 1
        else:
            settle_counter = 0

        if settle_counter >= settle_loops:
            print("COMPLETE!!!")
            if stop_at_end or exit_velocity_pct == 0.0:
                motor_Stop()
            else:
                # Chain mode: keep rolling at exit_velocity_pct so the next
                # move can start without a full stop (saves time in auton).
                forward = exit_velocity_pct if target[0] >= 0 else -exit_velocity_pct
                motorFLSpeed = forward + turnSpeed; motorFRSpeed = forward - turnSpeed
                motorBLSpeed = forward + turnSpeed; motorBRSpeed = forward - turnSpeed
                motor_Motion(motorFLSpeed, motorFRSpeed,
                             motorBLSpeed, motorBRSpeed)
            print(error); print(counter)

            return error

        elif counter > time_out:
            print("COMPLETE!!! Timed Out!!!")
            motor_Stop()
            print(error); print(counter)
            return error

        time.sleep(dt)
        counter += 1


# ============================================================
# AUTONOMOUS PID — TRACKING WHEEL-DRIVEN
# ============================================================

def autonomousPIDTracking(
    target,
    initialMaxSpeedLimit,
    maxSpeedLimit,
    time_out,
    export_flag,
    Kp_l,
    Kp_r,
    previousError=None,
    exit_velocity_pct=0.0,
    slow_down_distance=6.0,
    settle_error_dist=0.5,
    settle_error_heading=0.03,
    settle_loops=8,
    stop_at_end=True,
    heading_guard_deg=2.0,
    heading_guard_floor=0.0,
    tracking_wheel_factor=2 * math.pi,
):
    """Same PID algorithm as autonomousPID, but uses trackingWheelVert for
    distance feedback instead of drive motor encoders.

    This eliminates wheel-slip error from the feedback loop: the tracking
    wheel rolls on the field surface independently of the drive motors, so
    it reports actual travel even if the drive wheels slip.

    tracking_wheel_factor : inches per turn for trackingWheelVert.
                            Default 2*pi ≈ 6.28 matches a ~2-inch diameter wheel.
                            Update this value when ODO_FACTOR changes.

    The 'combine' global list is populated with per-loop telemetry when
    export_flag == 1; print it after the move to analyse tracking vs encoder.
    """

    global hook_flag, combine
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    initialVertDist = distVert.object_distance(INCHES)
    initialHorizDist = distHoriz.object_distance(INCHES)
    data_export = []
    combine     = []
    reset_tracking_wheels()   # zero tracking wheel at the start of each move

    tw_factor = tracking_wheel_factor if tracking_wheel_factor is not None else wheelFactor

    if previousError is None:
        previousError = [0.0, 0.0]

    Kp = Kp_l;  Ki = 0.0
    KpRotation = Kp_r;  KiRotation = 0.0
    Kd = 0.0;   KdRotation = 0.0  # increase if overshoot is a problem

    initial_heading = inertialSensor.rotation() * pi / 180.0
    currentPosition = [0.0, initial_heading]
    error           = [0.0, 0.0]
    integral        = 0.0
    headingIntegral = 0.0

    dt            = 0.005
    counter       = 0
    settle_counter = 0
    HEADING_GUARD_DEG = heading_guard_deg

    while True:
        error[0] = target[0] - currentPosition[0]
        error[1] = target[1] - currentPosition[1]

        integral        += error[0] * dt
        headingIntegral += error[1] * dt
        derivative        = (error[0] - previousError[0]) / dt
        headingDerivative = (error[1] - previousError[1]) / dt

        xOutput   = (Kp * error[0]) + (Ki * integral) + (Kd * derivative)
        turnSpeed = (KpRotation * error[1]
                     + KiRotation * headingIntegral
                     + KdRotation * headingDerivative)

        heading_error_deg = error[1] * 180.0 / pi
        abs_head_err = abs(heading_error_deg)
        if HEADING_GUARD_DEG > 0 and abs_head_err > HEADING_GUARD_DEG:
            scale = HEADING_GUARD_DEG / abs_head_err
            scale = max(heading_guard_floor, min(1.0, scale))
            xOutput *= scale

        motorFLSpeed = xOutput + turnSpeed; motorFRSpeed = xOutput - turnSpeed
        motorBLSpeed = xOutput + turnSpeed; motorBRSpeed = xOutput - turnSpeed

        distance_remaining = abs(error[0])
        exit_speed    = abs(exit_velocity_pct)
        dynamic_limit = maxSpeedLimit
        if slow_down_distance > 0 and abs(target[0]) > 0.001:
            blend = min(1.0, max(0.0, distance_remaining / slow_down_distance))
            dynamic_limit = exit_speed + (maxSpeedLimit - exit_speed) * blend

        if counter <= 100:
            #cap = initialMaxSpeedLimit
            cap = maxSpeedLimit
        else:
            cap = dynamic_limit

        maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                       abs(motorBLSpeed), abs(motorBRSpeed),
                       cap)
        if maxSpeed > cap:
            ratio = cap / maxSpeed
            motorFLSpeed *= ratio; motorFRSpeed *= ratio
            motorBLSpeed *= ratio; motorBRSpeed *= ratio

        motor_Motion(motorFLSpeed, motorFRSpeed,
                     motorBLSpeed, motorBRSpeed)

        previousError[0] = error[0]
        previousError[1] = error[1]

        # Update position: use vertical tracking wheel for distance, IMU for heading.
        # Subtract the arc the wheel sweeps due to rotation (wheel is offset from center).
        vertPos, _ = get_Tracking_Wheel_Position()
        currentPosition[1] = inertialSensor.rotation() * pi / 180.0
        heading_change = currentPosition[1] - initial_heading
        currentPosition[0] = vertPos * tw_factor + heading_change * VERT_WHEEL_LATERAL_OFFSET

        # Collect telemetry row when export_flag is on
        if counter % 5 == 0 and export_flag == 1:
            leftRotation, rightRotation = get_Rotation_Sensor_Position()
            encoderPos = ((leftRotation + rightRotation) / 2.0) * wheelFactor
            enc_heading_deg = calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr) * 180.0 / math.pi
            data_export = [counter, encoderPos, error[0], motorFLSpeed,
                    trackingWheelVertL.position(TURNS)*2*math.pi,
                    trackingWheelHorizR.position(TURNS)*2*math.pi,
                    distVert.object_distance(INCHES)-initialVertDist, distHoriz.object_distance(INCHES)-initialHorizDist,
                    currentPosition[1]*180/math.pi, error[1]*180/math.pi, headingDerivative,
                    enc_heading_deg]
            # data_export = [counter, error[0], derivative, error[1]*180/math.pi, headingDerivative,
            #                encoderPos, trackingWheelVertL.position(TURNS)*tracking_wheel_factor, trackingWheelHorizR.position(TURNS)*tracking_wheel_factor]
            combine += [data_export]

        if abs(error[0]) < settle_error_dist and abs(error[1]) < settle_error_heading:
            settle_counter += 1
        else:
            settle_counter = 0

        if settle_counter >= settle_loops:
            print("COMPLETE!!!")
            if stop_at_end or exit_velocity_pct == 0.0:
                motor_Stop()
            else:
                forward = exit_velocity_pct if target[0] >= 0 else -exit_velocity_pct
                motorFLSpeed = forward + turnSpeed; motorFRSpeed = forward - turnSpeed
                motorBLSpeed = forward + turnSpeed; motorBRSpeed = forward - turnSpeed
                motor_Motion(motorFLSpeed, motorFRSpeed,
                             motorBLSpeed, motorBRSpeed)
            print(error); print(counter)
            return error

        elif counter > time_out:
            print("COMPLETE!!! Timed Out!!!")
            motor_Stop()
            print(error); print(counter)
            return error

        time.sleep(dt)
        counter += 1


# ============================================================
# WALL-ALIGNMENT HELPERS
# ============================================================

def read_dist_mm_filtered(sensor, samples=3, sample_delay_ms=0,
                           min_mm=20, max_mm=2000, fallback=None):
    """Read a distance sensor multiple times and return the median.

    Taking a median over several samples filters out single-point spikes
    that can occur when the sensor sees the edge of an object or glare.
    If no valid reading is obtained (all out of [min_mm, max_mm]),
    the fallback value is returned instead (or max_mm if fallback is None).
    """
    values = []
    for _ in range(max(1, samples)):
        mm = sensor.object_distance(MM)
        if min_mm <= mm <= max_mm:
            values.append(mm)
        if sample_delay_ms > 0:
            wait(sample_delay_ms, MSEC)

    if values:
        values.sort()
        mid = len(values) // 2
        return values[mid] if len(values) % 2 == 1 else (values[mid - 1] + values[mid]) / 2.0

    return fallback if fallback is not None else max_mm


def finalize_to_front_wall(
    target_mm,
    timeout_ms=2500,
    kp=0.12,
    ki=0.0,
    kd=0.03,
    max_speed_pct=35.0,
    min_speed_pct=7.0,
    tolerance_mm=12.0,
    settle_loops=8,
    control_period_ms=5,
    sensor_period_ms=20,
    integral_limit=6000.0,
):
    """Creep the robot forward or backward until it is exactly target_mm
    from the front wall, using a PID loop driven by distVert.

    This is called at the end of an autonomous approach move to remove any
    distance error left by the encoder-PID (which can be off by 0.5–2 in).
    The robot must already be facing the wall roughly straight.

    sensor_period_ms is slower than control_period_ms because the distance
    sensor needs time between reads; the derivative term only updates when
    a fresh sensor reading is available.

    Returns the final distance error (mm) relative to target_mm.
    """
    dt          = control_period_ms / 1000.0
    integral    = 0.0
    settle_count = 0
    start_ms     = brain.timer.time(MSEC)
    last_sensor_ms = start_ms - sensor_period_ms
    last_dist    = read_dist_mm_filtered(distVert)
    last_error   = last_dist - target_mm

    while (brain.timer.time(MSEC) - start_ms) < timeout_ms:
        loop_start_ms = brain.timer.time(MSEC)

        # Only poll the sensor every sensor_period_ms to avoid noisy readings
        sensor_updated = False
        if (loop_start_ms - last_sensor_ms) >= sensor_period_ms:
            last_dist = read_dist_mm_filtered(distVert, fallback=last_dist)
            sensor_updated = True
            last_sensor_ms = loop_start_ms

        error = last_dist - target_mm

        # Integrate and clamp to prevent wind-up on a long approach
        integral += error * dt
        integral = max(-integral_limit, min(integral_limit, integral))

        # Derivative only updates with a fresh sensor reading
        if sensor_updated:
            deriv_dt   = sensor_period_ms / 1000.0 or dt
            derivative = (error - last_error) / deriv_dt
            last_error = error
        else:
            derivative = 0.0

        output = kp * error + ki * integral + kd * derivative
        output = max(-max_speed_pct, min(max_speed_pct, output))

        # Enforce a minimum speed so the robot doesn't stall near the target
        if abs(error) > tolerance_mm and abs(output) < min_speed_pct:
            output = min_speed_pct if output >= 0 else -min_speed_pct

        motor_Motion(output, output, output, output)

        if abs(error) <= tolerance_mm:
            settle_count += 1
        else:
            settle_count = 0

        if settle_count >= settle_loops:
            break

        remaining_ms = control_period_ms - (brain.timer.time(MSEC) - loop_start_ms)
        if remaining_ms > 0:
            wait(remaining_ms, MSEC)

    motor_Stop()
    return last_dist - target_mm


# ============================================================
# TERMINAL ASCII PLOT
# ============================================================

def plot_ascii(data, width=60, height=20):
    """Print a terminal scatter plot of linear position vs time.
    Each row of data is [counter, encoder_pos_in, ...].
    """
    if not data:
        print("plot_ascii: no data")
        return
    times = [r[0] for r in data]
    pos   = [r[1] for r in data]
    t_min, t_max = min(times), max(times)
    p_min, p_max = min(pos),   max(pos)
    t_rng = t_max - t_min or 1
    p_rng = p_max - p_min or 1

    grid = [[' '] * width for _ in range(height)]
    for t, p in zip(times, pos):
        c = int((t - t_min) / t_rng * (width - 1))
        r = height - 1 - int((p - p_min) / p_rng * (height - 1))
        grid[max(0, min(height - 1, r))][max(0, min(width - 1, c))] = '*'

    print("--- linear pos (in) vs time (iterations) ---")
    for i, row in enumerate(grid):
        label = p_max - i * p_rng / (height - 1)
        print("{:6.1f}|{}".format(label, ''.join(row)))
    print("      +" + "-" * width)
    left  = str(int(t_min))
    right = str(int(t_max))
    gap   = width - len(left) - len(right)
    print("      " + left + " " * max(0, gap) + right)
    print("      time (iterations) ->")


# ============================================================
# COMPETITION / PROGRAM ENTRY POINT
# ============================================================

vexcode_brain_precision   = 0
vexcode_console_precision = 0
myVariable                = 0


def vexcode_auton_function():
    """Called by the Competition object when autonomous mode starts.
    Spawns onauton_autonomous_0 as a background thread so the competition
    framework can kill it cleanly when the autonomous period ends.
    """
    auton_task_0 = Thread(onauton_autonomous_0)
    while competition.is_autonomous() and competition.is_enabled():
        wait(10, MSEC)
    auton_task_0.stop()


def when_started1():
    """Main autonomous routine.
    Calibrates the IMU (double-calibrates if the first attempt is too fast),
    then runs a 24-inch forward move as an example.
    Replace the autonomousPID call below with your actual auton sequence.
    """
    t_1 = brain.timer.time(MSEC)
    inertialSensor.calibrate()
    while inertialSensor.is_calibrating():
        wait(100, TimeUnits.MSEC)
    t_2 = brain.timer.time(MSEC)

    # IMU calibration should take at least 1800 ms; if it was shorter,
    # the sensor may not have settled — run a second calibration.
    if t_2 - t_1 < 1800:
        inertialSensor.calibrate()
        while inertialSensor.is_calibrating():
            wait(100, TimeUnits.MSEC)

    #toggle flips and the robot grabs onto the preload
    claw_close()
    wait(0.5,SECONDS)
    bias = -37

    # drives to the alliance goal
    Time_wait   = 400   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 10
    r           = 0
    v_min       = 60
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)

    # Time_wait   = 400   # observed max 178 iters; 2x buffer
    # export_flag = 1     # 1 = print debug telemetry
    # f           = 18     # TUNE: inches from start to ring 1
    # r           = 0+bias     # heading, facing straight into the ring
    # v_min       = 65    # ramp-up speed cap (%)
    # v_max       = 90    # cruise speed cap (%)
    # Kp_linear   = 6.8
    # Kp_rotation = 45

    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
    #               Kp_linear, Kp_rotation)
    # Time_wait   = 300   # observed max 127 iters; 2x buffer
    # export_flag = 1
    # f           = 0
    # r           = 90+bias
    # v_min       = 50
    # v_max       = 90
    # Kp_linear   = 6.8
    # Kp_rotation = 50

    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
    #               Kp_linear, Kp_rotation)

    # Time_wait   = 300   # observed max 120 iters; 2x buffer
    # export_flag = 1
    # f           = 6
    # r           = 90+bias
    # v_min       = 55
    # v_max       = 90
    # Kp_linear   = 6.8
    # Kp_rotation = 50

    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
    #               Kp_linear, Kp_rotation)
    # Scores pin in goal
    # Time_wait   = 200   # observed max 76 iters; 2x buffer
    # export_flag = 1
    # f           = -4
    # r           = 90+bias
    # v_min       = 55
    # v_max       = 90
    # Kp_linear   = 6.8
    # Kp_rotation = 50

    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
    #               Kp_linear, Kp_rotation)
    wait(0.5,SECONDS)
    Time_wait   = 400   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = -5
    r           = 0
    v_min       = 40
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    # goes to grab pin + cone

    Time_wait   = 250   # observed max 92 iters; 2x buffer
    export_flag = 1
    f           = 0
    r           = 145+bias
    v_min       = 65
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)

    Time_wait   = 400   # observed max 180 iters; 2x buffer
    export_flag = 1
    # f           = 17
    f           = 7
    r           = 145+bias
    v_min       = 65
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50
    # picks up pin + cone

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    wait (0.5,SECONDS)
    # goes to score in the neutral goal
    Time_wait   = 400   # observed max 180 iters; 2x buffer
    export_flag = 1
    f           = -19
    r           = 145+bias
    v_min       = 75
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    
    Time_wait   = 350   # observed max 158 iters; 2x buffer
    export_flag = 1
    f           = 0
    r           = 270+bias
    v_min       = 65
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)


    Time_wait   = 1200  # observed max 563 iters (outlier); 2x buffer
    export_flag = 1
    f           = 23
    r           = 270+bias
    v_min       = 75
    v_max       = 90
    Kp_linear   = 6
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    wait(0.5,SECONDS)
    
    Time_wait   = 200   # observed max 79 iters; 2x buffer
    export_flag = 1
    f           = -3
    r           = 270+bias
    v_min       = 55
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    # gets second pin + cone combo
    
    Time_wait   = 250   # observed max 90 iters; 2x buffer
    export_flag = 1
    f           = 0
    r           = 215+bias
    v_min       = 50
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    
    Time_wait   = 400   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 17
    r           = 215+bias
    v_min       = 75
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    wait (0.5,SECONDS)
    Time_wait   = 400   #observed max 178 iters; 2x buffer
    export_flag = 1
    f           = -5
    r           = 215+bias
    v_min       = 55
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    
    Time_wait   = 400   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 0
    r           = 290+bias
    v_min       = 62
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    wait(0.5,SECONDS)
    Time_wait   = 400   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = -8
    r           = 290+bias
    v_min       = 40
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    Time_wait   = 400   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 0
    r           = 270+bias
    v_min       = 40
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    
    Time_wait   = 1000   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 28
    r           = 270+bias
    v_min       = 40
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    
    # gets to final cone and picks it up
    Time_wait   = 900   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 8
    r           = 315+bias
    v_min       = 40
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    wait(0.5,SECONDS)
    Time_wait   = 900   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 0
    r           = 435+bias
    v_min       = 40
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    Time_wait   = 900   # observed max 178 iters; 2x buffer
    export_flag = 1
    f           = 12
    r           = 450+bias
    v_min       = 40
    v_max       = 90
    Kp_linear   = 6.8
    Kp_rotation = 50

    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait, export_flag,
                  Kp_linear, Kp_rotation)
    
    # Print the telemetry captured by autonomousPIDTracking (if it was used).
    # combine is only populated when autonomousPIDTracking is called; skip the
    # print safely when autonomousPID was used instead.

    # Headers = ["time", "error", "velocity", "rotation error",
    #            "rotational velocity", "encoder pos", "tw horz pos", "tw vert pos"]
    Headers=["time","encoder pos","encoder linear error","velocity","tw vert pos","tw horz pos",
             "distance vert pos", "distance horz pos",
             "rotation pos","rotation error","rotational velocity","enc rotation pos"]
    #Headers=["time","encoder pos","encoder error","odom pos","dist pos"]
    # data_export = [counter, encoderPos, error[0], derivative,
    #                 trackingWheelVert.position(TURNS)*2*math.pi,
    #                 trackingWheelHoriz.position(TURNS)*2*math.pi,
    #                 distVert.object_distance(INCHES)-initialVertDist, distHoriz.object_distance(INCHES)-initialHorizDist,
    #                 currentPosition[1]*180/math.pi, error[1]*180/math.pi, headingDerivative,
    #                 calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr) * 180.0 / math.pi]
    print(Headers)
    # wait(50,MSEC)
    # if combine:
    #     print(",".join(str(x) for x in [combine[-1][0], combine[-1][1], combine[-1][4], combine[-1][6]]))

    try:
        for row in combine:
            print(row)
            wait(50,MSEC)
        plot_ascii(combine)
    except NameError:
        pass
    wait(5000, MSEC)


def vexcode_driver_function():
    """Called by the Competition object during driver control.
    Extend this function with joystick-to-motor mappings for the driver period.
    """
    while competition.is_driver_control() and competition.is_enabled():
        wait(10, MSEC)


def onauton_autonomous_0():
    """Runs once when the program starts (before any competition mode begins).
    Set SENSOR_TEST near the bottom of this file to run a validation test
    outside of a competition match (e.g. during practice).
    """
    # if SENSOR_TEST is not None:
    #     run_sensor_validation(SENSOR_TEST)


# Register driver and autonomous callbacks with the competition manager,
# then call when_started1 for any pre-match setup or standalone testing.
competition = Competition(vexcode_driver_function, vexcode_auton_function)
when_started1()


# ============================================================
# DRIVETRAIN SENSOR VALIDATION  T1–T9  |  Override 2026-27
#
# PURPOSE
# -------
# Compare two position-tracking sensors side-by-side on every run:
#
#   BLUE  = Motor encoders   — integrated into each drive motor
#   GREEN = Odometry wheel   — dedicated tracking wheel (trackingWheelVert)
#
# Both sensors log simultaneously on the same physical run, eliminating
# run-to-run variability (battery level, tile conditions, start placement).
# Every difference in the logged values is caused by the sensor, not the trial.
#
# HOW TO RUN A TEST
# -----------------
# 1. Set SENSOR_TEST (at the bottom of this file) to the test ID, e.g. "t1".
# 2. Download and run the program.
# 3. Follow the on-screen prompts — press controller button A to gate each trial.
# 4. Read enc_reported and odo_reported from the console.
# 5. Measure actual position with a laser meter and fill in the record sheet.
#
# HARDWARE NOTE
# -------------
# odo_heading in T5 / T7 / T8 is reported as IMU reading because a single
# vertical tracking wheel cannot compute heading on its own (you would need
# a second parallel wheel).  enc_heading is computed from the left/right
# motor encoder differential.
# ============================================================

# Inches per turn for the vertical tracking wheel.
# Measure the wheel diameter and compute: math.pi * diameter
# 2.75-inch wheel  →  math.pi * 2.75 ≈ 8.64
# 2.00-inch wheel  →  math.pi * 2.00 ≈ 6.28  (matches autonomousPIDTracking default)
ODO_FACTOR = 2 * math.pi

# Horizontal distance (inches) from the robot centerline to the vertical tracking wheel.
# Used in heading estimation from tracking wheels: Δθ = Δvert / VERT_WHEEL_LATERAL_OFFSET
VERT_WHEEL_LATERAL_OFFSET = -0.75


# ------------------------------------------------------------------ #
#  Sensor-validation internal helpers  (prefixed _sv_)               #
# ------------------------------------------------------------------ #

def _sv_enc_in():
    """Return the average drive-encoder distance in inches since the last Reset_all."""
    l, r = get_Rotation_Sensor_Position()
    return ((l + r) / 2.0) * wheelFactor