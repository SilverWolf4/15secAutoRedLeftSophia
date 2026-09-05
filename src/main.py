# ---------------------------------------------------------------------------- #
#                                                                              #
#   Module:       main.py                                                      #
#   Author:       Margaret Liu                                                 #
#   Created:      1/13/2025, 10:24:50 PM                                       #
#   Description:  H-drive drivetrain with PID autonomous + sensor validation   #
#                                                                              #
# ---------------------------------------------------------------------------- #

import time
import math
from vex import *

# ============================================================
# ROBOT CONFIGURATION
# ============================================================

brain      = Brain()
controller_1 = Controller(PRIMARY)

# Half-track widths (center of left wheels to robot centerline,
# and center of right wheels to robot centerline), in inches.
# Used in the heading calculation: rotation = (left_arc - right_arc) / (tl + tr)
tl = 6.75   # left half-track width
tr = 6.75   # right half-track width

pi = 3.14159

# Converts motor shaft rotations (turns) to linear inches at the wheel.
# Formula: pi * wheel_diameter / gear_ratio
# Wheel diameter = 3.25 in, gear ratio = 36/24 = 1.5  →  pi * 3.25 / 1.5
wheelFactor = pi * 3.25 *3/4

# ---- Drive motors (H-drive: 3 left, 3 right + 1 center strafe) ----
# reversed=True  means the motor's positive direction spins the wheel forward
motorFL = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)   # front-left
motorFR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)  # front-right
motorBL = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)   # back-left
motorBR = Motor(Ports.PORT3,  GearSetting.RATIO_6_1, False)  # back-right
motorML = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)  # mid-left 
motorMR = Motor(Ports.PORT2,  GearSetting.RATIO_6_1, False)   # mid-right

clawRotationMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True) #claw rotation motor
clawOpen = DigitalOut(brain.three_wire_port.a)#open and close claw
clawTurn = DigitalOut(brain.three_wire_port.b) #turn claw 180 degrees

extensionL = Motor(Ports.PORT5, GearSetting.RATIO_36_1, True)   # left extension motor
extensionR = Motor(Ports.PORT6, GearSetting.RATIO_36_1, False) # right extension motor

# ---- Sensors ----
inertialSensor     = Inertial(Ports.PORT7)
trackingWheelVert  = Rotation(Ports.PORT4, True)  # vertical tracking wheel  (forward/back)
trackingWheelHoriz = Rotation(Ports.PORT17, True)   # horizontal tracking wheel (strafe)
distVert  = Distance(Ports.PORT12)  # vertical-facing distance sensor
distHoriz = Distance(Ports.PORT20)   # horizontal-facing distance sensor

# ---- Pneumatics ----
arm = DigitalOut(brain.three_wire_port.a)
cap = DigitalOut(brain.three_wire_port.h)

# ---- Distance Sensor ----

# ---- AI Vision (used for game element detection) ----
class GameElements:
    MOBILE_GOAL = 0
    RED_RING    = 1
    BLUE_RING   = 2
    SKIP        = -1

AI_clamp = AiVision(Ports.PORT15, AiVision.ALL_AIOBJS)


# ============================================================
# MOTOR HELPERS
# ============================================================
def extension_move(time_seconds,velocity):
    extensionL.set_stopping(HOLD)
    extensionR.set_stopping(HOLD)

    extensionL.spin(DirectionType.FORWARD, velocity, VelocityUnits.PERCENT)
    extensionR.spin(DirectionType.FORWARD, velocity, VelocityUnits.PERCENT)
    wait(time_seconds, SECONDS)

    extensionL.stop()
    extensionR.stop()

def extension_move_to_position(target_position_turns, velocity, waitCon=False):
    """Move the extension motors to a specific position (in degrees) at a given velocity."""
    extensionL.set_stopping(HOLD)
    extensionR.set_stopping(HOLD)
    extensionL.set_velocity(velocity, PERCENT)
    extensionR.set_velocity(velocity, PERCENT)
    extensionL.spin_to_position(target_position_turns, TURNS, wait=False)
    extensionR.spin_to_position(target_position_turns, TURNS, wait=waitCon)

def claw_go_to_front(degrees):
    #make sure that robot is in starting position before this (inspection position)
    # clawOpen.set(False)
    clawRotationMotor.set_velocity(60,PERCENT)
    clawRotationMotor.set_stopping(HOLD)
    clawRotationMotor.spin_to_position(degrees, DEGREES, wait=True)#220 working as of 8/17/26

def claw_go_to_back():
    #make sure that robot is in starting position before this (inspection position)
    clawTurn.set(False)
    clawOpen.set(False)
    
    clawRotationMotor.set_velocity(60,PERCENT)
    clawRotationMotor.set_stopping(HOLD)
    clawRotationMotor.spin_to_position(0, DEGREES, wait=False)
    wait(0.3,SECONDS)
    
def motor_Stop():
    """Coast all six drive motors to a stop."""
    motorFL.stop(); motorFR.stop()
    motorML.stop(); motorMR.stop()
    motorBL.stop(); motorBR.stop()

def motor_hold():
    """Lock all six drive motors in place (active braking)."""
    motorFL.stop(HOLD); motorFR.stop(HOLD)
    motorML.stop(HOLD); motorMR.stop(HOLD)
    motorBL.stop(HOLD); motorBR.stop(HOLD)

def motor_brake():
    """Apply friction braking to all six drive motors."""
    motorFL.stop(BRAKE); motorFR.stop(BRAKE)
    motorML.stop(BRAKE); motorMR.stop(BRAKE)
    motorBL.stop(BRAKE); motorBR.stop(BRAKE)

def motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed, motorMLSpeed, motorMRSpeed):
    """Spin all six drive motors at the given velocity percentages.
    Positive values = forward for each motor (reversed flag handled at init).
    """
    motorFL.spin(DirectionType.FORWARD, motorFLSpeed, VelocityUnits.PERCENT)
    motorFR.spin(DirectionType.FORWARD, motorFRSpeed, VelocityUnits.PERCENT)
    
    motorML.spin(DirectionType.FORWARD, motorMLSpeed, VelocityUnits.PERCENT)
    motorMR.spin(DirectionType.FORWARD, motorMRSpeed, VelocityUnits.PERCENT)
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
    motorML.set_position(0, DEGREES)
    motorMR.set_position(0, DEGREES)
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
    trackingWheelVert.set_position(0, TURNS)
    trackingWheelHoriz.set_position(0, TURNS)

def get_Tracking_Wheel_Position():
    """Return (vertical_turns, horizontal_turns) from the tracking wheels.
    Vertical wheel measures forward/backward travel.
    Horizontal wheel measures sideways (strafe) travel.
    Multiply by ODO_FACTOR to convert turns → inches.
    """
    return trackingWheelVert.position(TURNS), trackingWheelHoriz.position(TURNS)


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
    export_flag         : set to 1 to print debug telemetry every 6 loops.
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

    global combine
    combine = []
    initialVertDist = distVert.object_distance(INCHES)
    initialHorizDist = distHoriz.object_distance(INCHES)
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    Reset_all()   # zero encoders at the start of each move

    if previousError is None:
        previousError = [0.0, 0.0]

    # PID gains
    Kp = Kp_l          # linear proportional
    Ki = 0.0            # linear integral (disabled)
    KpRotation = Kp_r   # heading proportional
    KiRotation = 0.0    # heading integral (disabled)
    # Derivative gains — increase these if you see overshoot or oscillation
    Kd = 0.0
    KdRotation = 0.0

    # currentPosition tracks how far we've gone and what heading we're at
    currentPosition = [0.0, inertialSensor.rotation() * pi / 180.0]

    error           = [0.0, 0.0]  # [distance_error_in, heading_error_rad]
    integral        = 0.0         # accumulated distance error (for I term)
    headingIntegral = 0.0         # accumulated heading error  (for I term)

    dt             = 0.005   # loop period: 5 ms → 200 Hz
    counter        = 0
    settle_counter = 0
    leftRotation   = 0.0
    rightRotation  = 0.0

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

        # Pure in-place rotation (target[0]==0): ignore the linear term entirely.
        # Otherwise encoder drift picked up while spinning in place feeds back
        # into xOutput and fights the turn, preventing it from ever settling.
        if abs(target[0]) < 0.001:
            xOutput = 0.0

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
        motorMLSpeed = xOutput + turnSpeed
        motorMRSpeed = xOutput - turnSpeed
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
            speed_cap = initialMaxSpeedLimit
        else:
            speed_cap = dynamic_limit

        maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                       abs(motorMLSpeed), abs(motorMRSpeed),
                       abs(motorBLSpeed), abs(motorBRSpeed),
                       speed_cap)
        if maxSpeed > speed_cap:
            ratio = speed_cap / maxSpeed
            motorFLSpeed *= ratio; motorFRSpeed *= ratio
            motorMLSpeed *= ratio; motorMRSpeed *= ratio
            motorBLSpeed *= ratio; motorBRSpeed *= ratio

        # 7. Send speeds to the motors
        motor_Motion(motorFLSpeed, motorFRSpeed,
                     motorBLSpeed, motorBRSpeed,
                     motorMLSpeed, motorMRSpeed)

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

        # 10. Optional debug telemetry every 6 loops
        if counter % 6 == 0 and export_flag == 1:
            leftRotation, rightRotation = get_Rotation_Sensor_Position()
            encoderPos = ((leftRotation + rightRotation) / 2.0) * wheelFactor
            enc_heading_deg = calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr) * 180.0 / math.pi
            data_export = [counter, encoderPos, error[0], derivative,
                    trackingWheelVert.position(TURNS) * 2 * math.pi,
                    trackingWheelHoriz.position(TURNS) * 2 * math.pi,
                    distVert.object_distance(INCHES) - initialVertDist,
                    distHoriz.object_distance(INCHES) - initialHorizDist,
                    currentPosition[1] * 180 / math.pi, error[1] * 180 / math.pi, headingDerivative,
                    enc_heading_deg]
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
                motorMLSpeed = forward + turnSpeed; motorMRSpeed = forward - turnSpeed
                motorBLSpeed = forward + turnSpeed; motorBRSpeed = forward - turnSpeed
                motor_Motion(motorFLSpeed, motorFRSpeed,
                             motorBLSpeed, motorBRSpeed,
                             motorMLSpeed, motorMRSpeed)
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
    settle_error_heading=0.05,#previously 0.03
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

    global combine
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    initialVertDist = distVert.object_distance(INCHES)
    initialHorizDist = distHoriz.object_distance(INCHES)
    combine = []
    reset_tracking_wheels()   # zero tracking wheel at the start of each move

    tw_factor = tracking_wheel_factor if tracking_wheel_factor is not None else wheelFactor

    if previousError is None:
        previousError = [0.0, 0.0]

    Kp = Kp_l;  Ki = 0.0
    KpRotation = Kp_r;  KiRotation = 0.0
    Kd = 0.0;   KdRotation = 0.5  # increase if overshoot is a problem (prev 0)

    currentPosition = [0.0, inertialSensor.rotation() * pi / 180.0]
    error           = [0.0, 0.0]
    integral        = 0.0
    headingIntegral = 0.0

    dt             = 0.005
    counter        = 0
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

        # Pure in-place rotation (target[0]==0): ignore the linear term entirely.
        # Otherwise phantom distance picked up by the offset tracking wheel while
        # spinning in place feeds back into xOutput and fights the turn, which
        # keeps regenerating error[0] and prevents the move from ever settling.
        if abs(target[0]) < 0.001:
            xOutput = 0.0

        heading_error_deg = error[1] * 180.0 / pi
        abs_head_err = abs(heading_error_deg)
        if HEADING_GUARD_DEG > 0 and abs_head_err > HEADING_GUARD_DEG:
            scale = HEADING_GUARD_DEG / abs_head_err
            scale = max(heading_guard_floor, min(1.0, scale))
            xOutput *= scale

        motorFLSpeed = xOutput + turnSpeed; motorFRSpeed = xOutput - turnSpeed
        motorMLSpeed = xOutput + turnSpeed; motorMRSpeed = xOutput - turnSpeed
        motorBLSpeed = xOutput + turnSpeed; motorBRSpeed = xOutput - turnSpeed

        distance_remaining = abs(error[0])
        exit_speed    = abs(exit_velocity_pct)
        dynamic_limit = maxSpeedLimit
        if slow_down_distance > 0 and abs(target[0]) > 0.001:
            blend = min(1.0, max(0.0, distance_remaining / slow_down_distance))
            dynamic_limit = exit_speed + (maxSpeedLimit - exit_speed) * blend

        if counter <= 100:
            speed_cap = initialMaxSpeedLimit
        else:
            speed_cap = dynamic_limit

        maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                       abs(motorMLSpeed), abs(motorMRSpeed),
                       abs(motorBLSpeed), abs(motorBRSpeed),
                       speed_cap)
        if maxSpeed > speed_cap:
            ratio = speed_cap / maxSpeed
            motorFLSpeed *= ratio; motorFRSpeed *= ratio
            motorMLSpeed *= ratio; motorMRSpeed *= ratio
            motorBLSpeed *= ratio; motorBRSpeed *= ratio

        motor_Motion(motorFLSpeed, motorFRSpeed,
                     motorBLSpeed, motorBRSpeed,
                     motorMLSpeed, motorMRSpeed)

        previousError[0] = error[0]
        previousError[1] = error[1]

        # Update position: use vertical tracking wheel for distance, IMU for heading
        vertPos, _ = get_Tracking_Wheel_Position()
        currentPosition[0] = vertPos * tw_factor
        currentPosition[1] = inertialSensor.rotation() * pi / 180.0

        # Collect telemetry row when export_flag is on
        if counter % 5 == 0 and export_flag == 1:
            leftRotation, rightRotation = get_Rotation_Sensor_Position()
            encoderPos = ((leftRotation + rightRotation) / 2.0) * wheelFactor
            enc_heading_deg = calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr) * 180.0 / math.pi
            data_export = [counter, encoderPos, error[0], motorFLSpeed,
                    trackingWheelVert.position(TURNS) * 2 * math.pi,
                    trackingWheelHoriz.position(TURNS) * 2 * math.pi,
                    distVert.object_distance(INCHES) - initialVertDist,
                    distHoriz.object_distance(INCHES) - initialHorizDist,
                    currentPosition[1] * 180 / math.pi, error[1] * 180 / math.pi, headingDerivative,
                    enc_heading_deg]
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
                motorMLSpeed = forward + turnSpeed; motorMRSpeed = forward - turnSpeed
                motorBLSpeed = forward + turnSpeed; motorBRSpeed = forward - turnSpeed
                motor_Motion(motorFLSpeed, motorFRSpeed,
                             motorBLSpeed, motorBRSpeed,
                             motorMLSpeed, motorMRSpeed)
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
# WALL-ALIGNMENT PID
# ============================================================

def read_dist_mm_filtered(sensor, samples=3, sample_delay_ms=0,
                           min_mm=20, max_mm=2000, fallback=None):
    """Read a distance sensor multiple times and return the median.

    Taking a median over several samples filters out single-point spikes
    that can occur when the sensor sees the edge of an object or glare.
    If no valid reading is obtained (all out of [min_mm, max_mm]),
    `fallback` is returned instead (default None, meaning "no trustworthy
    reading" -- callers must handle that rather than treating it as a
    real distance).
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

    return fallback


def finalize_to_front_wall(
    target_mm,
    timeout_ms=700,
    kp=0.05,
    ki=0.0,
    kd=0.03,
    max_speed_pct=10.0,
    min_speed_pct=3.0,
    tolerance_mm=8.0,
    settle_loops=8,
    control_period_ms=5,
    sensor_period_ms=20,
    integral_limit=6000.0,
    min_mm=20,
    max_mm=500,
):
    """Creep the robot forward or backward until it is exactly target_mm
    from the front wall, using a PID loop driven by distVert.

    This is called at the end of an autonomous approach move to remove any
    distance error left by the encoder-PID (which can be off by 0.5–2 in).
    The robot must already be facing the wall roughly straight.

    sensor_period_ms is slower than control_period_ms because the distance
    sensor needs time between reads; the derivative term only updates when
    a fresh sensor reading is available.

    min_mm/max_mm bound which readings are trusted. max_mm is kept well
    under the sensor's full 2000mm range because this is only ever called
    at close range (after an approach move) -- a reading anywhere near the
    far end almost always means the sensor missed a thin/round target (e.g.
    a mobile goal pole) and is instead seeing the field wall behind it, not
    that the target is genuinely far away.

    If no trustworthy reading is available (readings out of range, e.g. the
    robot is already touching the target and below the sensor's ~20mm
    minimum), the robot holds its current position instead of guessing --
    driving on a fabricated distance is how it used to ram straight through
    the target.

    Returns the final distance error (mm) relative to target_mm.
    """
    dt           = control_period_ms / 1000.0
    integral     = 0.0
    settle_count = 0
    start_ms       = brain.timer.time(MSEC)
    last_sensor_ms = start_ms - sensor_period_ms
    last_dist      = read_dist_mm_filtered(distVert, min_mm=min_mm, max_mm=max_mm)
    if last_dist is None:
        last_dist = target_mm   # no trustworthy reading yet -- assume settled, don't drive blind
    last_error     = last_dist - target_mm

    while (brain.timer.time(MSEC) - start_ms) < timeout_ms:
        loop_start_ms = brain.timer.time(MSEC)

        # Only poll the sensor every sensor_period_ms to avoid noisy readings
        sensor_updated = False
        if (loop_start_ms - last_sensor_ms) >= sensor_period_ms:
            fresh = read_dist_mm_filtered(distVert, min_mm=min_mm, max_mm=max_mm)
            if fresh is not None:
                last_dist = fresh
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

        motor_Motion(output, output, output, output, output, output)

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
    elapsed_ms = brain.timer.time(MSEC) - start_ms
    settled = settle_count >= settle_loops
    print("finalize_to_front_wall: {} after {}ms, error={:.1f}mm".format(
        "settled" if settled else "TIMED OUT", elapsed_ms, last_dist - target_mm))
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

def vexcode_auton_function():
    auton_task_0 = Thread(onauton_autonomous_0)
    while competition.is_autonomous() and competition.is_enabled():
        wait(10, MSEC)
    auton_task_0.stop()


def onauton_autonomous_0():
    global combine
    """Main autonomous routine.
    """
    #set stopping to hold for stablility
    #end
    clawRotationMotor.spin_to_position(30)
    #starting position is at a 30 deg angle offset to meet requirements
    #turn -30 degrees to face toggle wall
    r_offset=0
    Time_wait   = 70   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = -30      # target heading (degrees)
    v_min       = 60     # ramp-up speed cap (%)
    v_max       = 60     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    
    #bump into wall to flip toggle for the first time    
    Time_wait   = 50   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 5     # target distance (inches)
    r           = -30      # target heading (degrees)
    v_min       = 100     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #go forward, away from the toggle
    Time_wait   = 100   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -30     # target distance (inches)
    r           = -30      # target heading (degrees)
    v_min       = 100     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #flip toggle for the second time  
    Time_wait   = 100   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 5     # target distance (inches)
    r           = -30      # target heading (degrees)
    v_min       = 199     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    Time_wait   = 100   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -30     # target distance (inches)
    r           = -30      # target heading (degrees)
    v_min       = 100     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #reset rotation to zero as robot is alligned with wall.
    inertialSensor.set_rotation(0, DEGREES)

    #go away from wall to start scoring route
    Time_wait   = 350   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 15.75     # target distance (inches)
    r           = 0      # target heading (degrees)
    v_min       = 50     # ramp-up speed cap (%)
    v_max       = 70     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    extension_move_to_position(0.3,50)
    clawRotationMotor.set_velocity(100)

    # clawRotationMotor.spin_to_position(120)
    clawRotationMotor.spin_to_position(40)
    clawRotationMotor.set_stopping(HOLD)
    #raise arm to score preload
    # extension_move_to_position(0.3,100)
    clawOpen.set(False)
    # claw_go_to_front(10)
    
    #face red goal to score preload
    Time_wait   = 200   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 60     # ramp-up speed cap (%)
    v_max       = 75     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #go to red goal to score preload
    Time_wait   = 250   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 18     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 40     # ramp-up speed cap (%)
    v_max       = 60     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #use distance sensor to allign with loader vertically to score preload
    finalize_to_front_wall(90)
    extension_move_to_position(0,100)
    
    #scoring preload
    wait(0.1,SECONDS)
    clawOpen.set(True)
    # wait(0.1,SECONDS)

    #back up from red goal after scoring preload
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -6.75     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 50     # ramp-up speed cap (%)
    v_max       = 70     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    # turn to the cone + pin on the wall
    Time_wait   = 200   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    # r           = 133      # target heading (degrees)
    r           = 140      # target heading (degrees)
    v_min       = 50     # ramp-up speed cap (%)
    v_max       = 70     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation,
                           settle_error_heading=0.09, settle_loops=2)
    clawRotationMotor.spin_to_position(30)
    clawOpen.set(True)
        # turn to the cone + pin on the wall
    Time_wait   = 400   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 24     # target distance (inches)
    # r           = 133      # target heading (degrees)
    r           = 140      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 30     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    wait(0.1,SECONDS)
    clawOpen.set(False)
    # wait(0.1,SECONDS)
    extension_move_to_position(0.8,100)
    clawRotationMotor.spin_to_position(40)
    Time_wait   = 350   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -18.25     # target distance (inches)
    r           = 140      # target heading (degrees)
    v_min       = 20     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    Time_wait   = 200   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 50     # ramp-up speed cap (%)
    v_max       = 70     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation,
                           settle_error_heading=0.09, settle_loops=2)
    Time_wait   = 150   # loop iteration timeout (drives into wall on purpose, will always time out)
    export_flag = 0      # 1 = print debug telemetry
    f           = 8     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 40     # ramp-up speed cap (%)
    v_max       = 60     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    finalize_to_front_wall(80)
    extension_move_to_position(0,100)
    # extension_move_to_position(0.3,100)

    #scoring pin + cone
    wait(0.1,SECONDS)
    clawOpen.set(True)
    # wait(0.2,SECONDS)    
    return
    #use distance sensor to align better
    finalize_to_front_wall(40)

    #scoring loader cup and pin
    extension_move_to_position(0.6,50,waitCon=True)
    wait(0.2,SECONDS)
    clawOpen.set(True)
    wait(0.2,SECONDS)

    #go backwards from goal
    Time_wait   = 400   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -17     # target distance (inches)
    r           = 268      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 30     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #turn to face midfield
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 315      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 30     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #go to midfield and stop
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 60     # target distance (inches)
    r           = 315      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 30     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    return

    #go away from goal
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 13     # target distance (inches)
    r           = 0      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation) 

    #turn to 90 deg to face right wall
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)   
    claw_go_to_back()

    #go to right wall
    Time_wait   = 500   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 50     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 20     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 25
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    #use distance sensor to allign with loader
    finalize_to_front_wall(170)

    #face cup and pin
    Time_wait   = 200   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 225      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    claw_go_to_front(240)
    clawOpen.set(True)
    wait(0.5,SECONDS)
    finalize_to_front_wall(82)
    clawOpen.set(False)
    extension_move_to_position(3,100)
    return
    claw_go_to_front(250)
    clawOpen.set(True)
    #go into loader
    Time_wait   = 400   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 24     # target distance (inches)
    r           = 180      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 20     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    
    clawOpen.set(False)
    wait(0.5,SECONDS)
    #go out of loader
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -12     # target distance (inches)
    r           = 180      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 30     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    
    extension_move_to_position(3,70)
    wait(2,SECONDS)
    #turn 90 deg cw to face goal
    Time_wait   = 200   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 270      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 30     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #go to goal
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 30     # target distance (inches)
    r           = 270      # target heading (degrees)
    v_min       = 20     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    finalize_to_front_wall(50)
    clawOpen.set(True)
    return
    claw_go_to_back()
    #go to midfield
    Time_wait   = 1000   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 55     # target distance (inches)
    r           = 0      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    
    #turn away from goal
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 180      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    
    #go forward
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -20     # target distance (inches)
    r           = 180      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #turn to get closer to loader/ face wall
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 280      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    #hit wall to reset heading
    Time_wait   = 300   # loop iteration timeout
    export_flag = 0      # 1 = print debug telemetry
    f           = -65     # target distance (inches)
    r           = 280      # target heading (degrees)
    v_min       = 60     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    inertialSensor.set_rotation(270, DEGREES)
    return
    #turn to face red goal
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 30     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    extension_move_to_position(0.9, 10)
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #move towards red goal
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 20     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 40     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #go back after bump
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = -5.2     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 20     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #score
    extension_move_to_position(0, 50)
    wait(2,SECONDS)
    clawOpen.set(True)
    

    #back away from red goal
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = -9     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 30     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)



    #turn to face cup and pin
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = -13.5      # target heading (degrees)
    v_min       = 30     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    
    #go to cup and pin
    r_offset=0
    Time_wait   = 1000   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 15     # target distance (inches)
    r           = -13.5      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 12     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #grab cup and pin
    clawOpen.set(False)
    extension_move_to_position(6, 60)
    wait(3,SECONDS)

    #heading back to red goal
    r_offset=0
    Time_wait   = 700   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = -16     # target distance (inches)
    r           = -13.5      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 20     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 20     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #move towards red goal overshoot
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 15     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 30     # ramp-up speed cap (%)
    v_max       = 50     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    #go back
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = -5.2     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 30     # ramp-up speed cap (%)
    v_max       = 50     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #score
    extension_move_to_position(0, 50)
    wait(2,SECONDS)
    clawOpen.set(True)
    
    #exit
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = -15     # target distance (inches)
    r           = 90      # target heading (degrees)
    v_min       = 30     # ramp-up speed cap (%)
    v_max       = 50     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #allign with toggle
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 0      # target heading (degrees)
    v_min       = 30     # ramp-up speed cap (%)
    v_max       = 50     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #push toggle
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = -30     # target distance (inches)
    r           = 0      # target heading (degrees)
    v_min       = 100     # ramp-up speed cap (%)
    v_max       = 100     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    #go to mid field
    r_offset=0
    Time_wait   = 1000   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 65     # target distance (inches)
    r           = 0      # target heading (degrees)
    v_min       = 20     # ramp-up speed cap (%)
    v_max       = 60     # cruise speed cap (%) 
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    return
    #turn
    r_offset=0
    Time_wait   = 600   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 45      # target heading (degrees)
    v_min       = 30     # ramp-up speed cap (%)
    v_max       = 50     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)
    #go to cup and pin
    r_offset=0
    Time_wait   = 1000   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 18     # target distance (inches)
    r           = 41      # target heading (degrees)
    v_min       = 10     # ramp-up speed cap (%)
    v_max       = 20     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)

    clawOpen.set(False)

    #go forward to allign with red goal
    r_offset=0
    Time_wait   = 1000   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 7     # target distance (inches)
    r           = 43      # target heading (degrees)
    v_min       = 5     # ramp-up speed cap (%)
    v_max       = 10     # cruise speed cap (%) 
    Kp_linear   = 4
    Kp_rotation = 30
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)


    print("temperature:", motorFL.temperature())
    return
    # Example move: drive forward 10 inches
    r_offset=-37
    Time_wait   = 1000   # loop iteration timeout
    export_flag = 1      # 1 = print debug telemetry
    f           = 0     # target distance (inches)
    r           = 0      # target heading (degrees)
    v_min       = 20     # ramp-up speed cap (%)
    v_max       = 50     # cruise speed cap (%)
    Kp_linear   = 4
    Kp_rotation = 30
    extension_move_to_position(4, 10)
    wait(5, SECONDS)
    return
    autonomousPIDTracking([f, math.radians(r+r_offset)], v_min, v_max, Time_wait, export_flag,
                           Kp_linear, Kp_rotation)# test linear forward 
    wait(500, MSEC)
    print(trackingWheelVert.position(TURNS))
    wait(500, MSEC)




def vexcode_driver_function():
    """Called by the Competition object during driver control.
    Extend this function with joystick-to-motor mappings for the driver period.
    """
    while competition.is_driver_control() and competition.is_enabled():
        wait(10, MSEC)


def when_started1():
    """Runs once when the program starts (before any competition mode begins)."""
    # Calibrate the IMU here so it's ready before autonomous begins, rather
    # than eating into autonomous time.
    inertialSensor.calibrate()
    while inertialSensor.is_calibrating():
        wait(100, TimeUnits.MSEC)


# Register driver and autonomous callbacks with the competition manager,
# then call when_started1 for any pre-match setup or standalone testing.
competition = Competition(vexcode_driver_function, vexcode_auton_function)
when_started1()
