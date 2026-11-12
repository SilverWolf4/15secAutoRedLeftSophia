#include "main.h"
#include "configs.h"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    printf("hello from the robot!\n");
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            printf("X: %.2f  Y: %.2f  Theta: %.2f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            pros::delay(100);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Drives toward a pin, slowing down as the distance sensor gets closer,
 * then stops and closes the claw.
 *
 * targetHeading: heading (degrees) to hold while driving
 * targetDist:    distance (mm) at which to stop and close the claw
 * timeout:       give up after this many ms
 */
void grabPin(double targetHeading, double targetDist = 60, int timeout = 1500) {
    const double kP = 0.25;          // speed per mm of distance left
    const double kD = 0.7;           // damps the speed if closing in too fast
    const double maxSpeed = 20;      // speed when the pin is far away
    const double minSpeed = 12;      // lowest speed, so the robot doesn't stall
    const double headingKP = 2.0;    // keeps the robot driving straight

    double prevError = 0;
    bool first = true;
    int start = pros::millis();
    while (pros::millis() - start < timeout) {
        int d = distance_sensor.get();          // mm (9999 if nothing seen)
        if (d <= 0 || d > 600) d = 600;         // nothing seen yet: treat as far away

        double error = d - targetDist;
        if (error <= 0) break;                  // reached the pin

        double derivative = first ? 0 : error - prevError;
        prevError = error;
        first = false;

        double speed = kP * error + kD * derivative;
        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < minSpeed) speed = minSpeed;

        double turn = headingKP * (targetHeading - chassis.getPose().theta);

        left_motor_group.move(speed + turn);
        right_motor_group.move(speed - turn);
        pros::delay(10);
    }
    // Stop the drive before closing the claw so the pin isn't pushed over
    left_motor_group.brake();
    right_motor_group.brake();
    claw.set_value(false); // close the claw
    pros::delay(150);      // let the claw grip (was 250)
    slide.move_absolute(360, 127); // lift the pin
                         // let the claw grip
}

/**
 * Waits for the current motion, but ends it early once the robot has stopped
 * moving (e.g. it is pushed up against a goal). Without this, a motion whose
 * target is "inside" the goal keeps pushing until its full timeout runs out.
 *
 * minTime:    always let the motion run at least this long (ms)
 * stallSpeed: robot counts as stopped below this speed (inches per second)
 * stallTime:  how long it has to stay stopped before we cancel (ms)
 */
void waitUntilStalled(int minTime = 300, double stallSpeed = 4, int stallTime = 150) {
    int start = pros::millis();
    int stillSince = pros::millis();
    lemlib::Pose last = chassis.getPose();
    while (chassis.isInMotion()) {
        pros::delay(10);
        lemlib::Pose now = chassis.getPose();
        if (now.distance(last) > stallSpeed * 0.01) stillSince = pros::millis(); // still moving
        last = now;
        if (pros::millis() - start > minTime && pros::millis() - stillSince > stallTime) {
            chassis.cancelMotion();
            break;
        }
    }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    //set position to x:0, y:0, heading:0
    
    /*toggling*/
    chassis.setPose(0, 0, 0);

    slide.move_absolute(600, 127);
    //first toggle
    chassis.turnToHeading(-40, 600);
    chassis.waitUntilDone();

    chassis.setPose(0, 0, 0);
    chassis.waitUntilDone();
    
    chassis.moveToPoint(0, 2, 200);
    chassis.waitUntilDone();
    //second toggle
    chassis.moveToPoint(0, -10, 300,{.forwards=false,.minSpeed=127});
    chassis.waitUntilDone();
    
    chassis.moveToPoint(0, 2, 300);
    chassis.moveToPoint(0, -10, 300,{.forwards=false,.minSpeed=127});
    
    /*going to red goal to score preload*/
    // No minSpeed, so the robot fully settles at the pose instead of exiting while still moving
    chassis.moveToPose(18, 14, 90, 1500, {.lead = 0.4});
    chassis.waitUntilDone();

    // Square up to 90 degrees so the push below is straight
    chassis.turnToHeading(90, 250);
    chassis.waitUntilDone();

    // Push straight into the goal: aim at the current y so the robot drives along
    // its heading instead of turning toward (19, 15). Slower speed = repeatable stop.
    chassis.moveToPoint(25, chassis.getPose().y, 200);
    chassis.waitUntilDone();
    
    chassis.setPose(17.3, 14.8,chassis.getPose().theta); // reset position to the center of the goal
    chassis.waitUntilDone();
    
    //scoring preload
    slide.move_absolute(0, 127);
    pros::delay(100);

    claw.set_value(true);
    pros::delay(200);
    
    //backing out of red goal after scoring preload
    chassis.moveToPoint(1, 15, 2000,{.forwards = false});
    chassis.waitUntilDone();

    /*first ground pin*/
    // Face the pin. From (0, 15) to (17.4, 42.08) the heading is about 32.7°, not 37.8°
    chassis.turnToHeading(48, 500);
    chassis.waitUntilDone();

    // Drive fast to a point about 6" before the pin
    chassis.moveToPose(19.3, 32.1, 48,1500,{.maxSpeed=70,.minSpeed=10});
    chassis.waitUntilDone();
    
    // Drive toward the first ground pin, slowing down as the distance sensor gets closer
    grabPin(48,60);


    // Only lift once the pin is gripped
    slide.move_absolute(3*360, 127);
    
    chassis.moveToPoint(28, 39, 1500, {.maxSpeed = 80});

    //scoring first ground pin
    chassis.turnToHeading(180, 500);
    chassis.waitUntilDone();

    // Robot stops against the goal before reaching the target, so end as soon as it stalls
    chassis.moveToPose(30, 19, 180, 1000);
    waitUntilStalled();

    slide.move_absolute(0, 100);
    pros::delay(300);
    claw.set_value(true);

    /*second ground pin*/
    pros::delay(200);
    chassis.setPose(30,21.6,chassis.getPose().theta);
    chassis.waitUntilDone();

    chassis.turnToHeading(180, 300);
    chassis.waitUntilDone();

    //heading to second ground pin
    chassis.moveToPose(30, 40, 180, 1000,{.forwards=false});
    chassis.waitUntilDone();

    chassis.turnToHeading(133, 500);
    chassis.waitUntilDone();

    chassis.moveToPose(48.6, 19.5, 133, 700,{.maxSpeed=70, .minSpeed=10});
    chassis.waitUntilDone();

    //grabbing second ground pin
    grabPin(133, 60, 1000);
    slide.move_absolute(5*360, 127);
    chassis.moveToPose(58.6, 12, 133, 1000);
    chassis.waitUntilDone();

    chassis.turnToHeading(270, 400);
    chassis.waitUntilDone();

    //going to goal
    // Target x=30 is inside the goal (robot stops at ~x=40), so end as soon as it stalls
    chassis.moveToPose(30, 12, 270, 1000);
    waitUntilStalled();

    chassis.setPose(40, 13, chassis.getPose().theta);
    chassis.waitUntilDone();

    //scoring second ground pin
    // Slide 80 -> 127 (~1.6x faster), so the delay is cut by the same ratio (700 -> 440)
    // to open the claw at the same slide height as before
    slide.move_absolute(1*360, 127);
    pros::delay(440);
    claw.set_value(true);
    pros::delay(100);
    chassis.waitUntilDone();
    //return here for 15second auto!!!
    return;

    /*loader first pin*/
    //going towards loader to grab first pin
    chassis.moveToPose(64, 13, 270, 1500,{.forwards=false,.minSpeed=40});
    chassis.waitUntilDone();
    
    
    slide.move_absolute(0, 127);
    chassis.turnToHeading(180, 400,{.maxSpeed=50});
    chassis.waitUntilDone();

    //alligning with loader and grabbing first pin from loader
    chassis.moveToPose(65, -3, 180, 1000);
    chassis.waitUntilDone();

    claw.set_value(false);
    pros::delay(200);

    //allign with red goal to score first loader pin
    chassis.moveToPose(64, 13, 180, 1000,{.forwards=false});
    chassis.waitUntilDone();

    slide.move_absolute(6*360, 127);
    pros::delay(200);

    //turn to face red goal to score first loader pin
    chassis.turnToHeading(270, 400,{.maxSpeed=50});
    chassis.waitUntilDone();

    //scoring first loader pin
    chassis.moveToPose(30, 12, 270, 1500);
    chassis.waitUntilDone();

    chassis.setPose(40, 13, chassis.getPose().theta);
    chassis.waitUntilDone();

    slide.move_absolute(3*360, 70);
    pros::delay(700);
    claw.set_value(true);
    pros::delay(100);
    chassis.waitUntilDone();

    /*loader second pin*/
    //going towards loader to grab second pin
    chassis.moveToPose(64, 13, 270, 1500,{.forwards=false,.minSpeed=40});
    chassis.waitUntilDone();
    
    
    slide.move_absolute(0, 127);
    chassis.turnToHeading(180, 400,{.maxSpeed=50});
    chassis.waitUntilDone();

    //alligning with loader and grabbing second pin from loader
    chassis.moveToPose(65, -3, 180, 1000);
    chassis.waitUntilDone();

    claw.set_value(false);
    pros::delay(200);

    //allign with red goal to score second loader pin
    chassis.moveToPose(64, 13, 180, 1000,{.forwards=false});
    chassis.waitUntilDone();

    slide.move_absolute(7*360, 127);
    pros::delay(200);

    //turn to face red goal to score second loader pin
    chassis.turnToHeading(270, 400,{.maxSpeed=50});
    chassis.waitUntilDone();

    //scoring second loader pin
    chassis.moveToPose(30, 12, 270, 1500);
    chassis.waitUntilDone();

    chassis.setPose(40, 13, chassis.getPose().theta);
    chassis.waitUntilDone();

    slide.move_absolute(3*360, 70);
    pros::delay(700);
    claw.set_value(true);
    pros::delay(100);
    chassis.waitUntilDone();

    /*loader singular pin to score on second goal*/
    //going towards loader to grab singular pin
    chassis.moveToPose(64, 13, 270, 1500,{.forwards=false,.minSpeed=40});
    chassis.waitUntilDone();
    
    
    slide.move_absolute(0, 127);
    chassis.turnToHeading(180, 400,{.maxSpeed=50});
    chassis.waitUntilDone();

    //alligning with loader and grabbing second pin from loader
    chassis.moveToPose(65, -3, 180, 1000);
    chassis.waitUntilDone();

    claw.set_value(false);
    pros::delay(200);

    //allign with red goal to score second loader pin
    chassis.moveToPose(64, 13, 180, 1000,{.forwards=false});
    chassis.waitUntilDone();

    //face second red goal to score singular loader pin
    chassis.turnToHeading(320, 400,{.maxSpeed=50});
    chassis.waitUntilDone();

    //go to second red goal to score singular loader pin
    chassis.moveToPose(50, 40, 0, 1000);//untested
    chassis.waitUntilDone();
    pros::lcd::print(3, "testt");
    return;
    
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    bool clawState = false;
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.curvature(leftY, rightX);
        // move the slide
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            slide.move(127);    // Move lift up when pressed
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            slide.move(-127);   // Move lift down pressed
        } else {
            slide.move(0);      // Holds the lift position when nothing is pressed
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            clawState = !clawState; // Flip the logical state so that claw state becomes opposite every L1 press
            claw.set_value(clawState);
        }
        pros::delay(25); // delay to prevent overloading
    }
}