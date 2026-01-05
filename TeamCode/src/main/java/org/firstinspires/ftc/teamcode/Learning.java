/*
 * TeachMode_Commented.java
 *
 * A fully-commented "teaching" OpMode for beginners ("bengineers").
 * This OpMode is disabled so it will NOT run on the robot. It contains
 * plain, line-by-line explanations and commented example code that
 * shows exactly how to:
 *  1) Drive forward a specific linear distance using encoders.
 *  2) Rotate (turn) the robot by a specified angle using encoders.
 *
 * Nothing in this file will move your robot because all actionable
 * code examples are in comments. Read the comments, copy the snippets
 * you want into a real autonomous OpMode, and test carefully.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/* Mark this OpMode as Disabled so it does not appear in the driver station.
   We purposely disable it because its only job is teaching. */
@Disabled
@Autonomous(name = "TeachMode_Commented", group = "Teaching")
public class Learning extends OpMode {

    /*
     * ---------- HIGH-LEVEL NOTES (read first) ----------
     * - We use motor encoders to run a specific distance or rotate a specific angle.
     * - To drive forward a distance:
     *     1) compute target encoder ticks from distance (inches/mm -> ticks)
     *     2) set each drive motor's target position
     *     3) set motors to RUN_TO_POSITION
     *     4) set motor power to a positive value to start motion
     *     5) wait until motors reach position (or use a timer tolerance)
     * - To rotate (turn), you set the left & right targets to opposite signs
     *   so the wheels turn in opposite directions and the robot spins in place.
     *
     * All the code below is commented out on purpose — copy-paste into a real
     * OpMode and remove comments to run.
     *
     * ---------- NAMES USED BELOW ----------
     * left_drive  --> the left drive motor (hardware name: "left_drive")
     * right_drive --> the right drive motor (hardware name: "right_drive")
     *
     * ---------- Units ----------
     * - Distances can be expressed in inches or mm; convert consistently.
     * - Angles in degrees are converted to radians for the math.
     *
     * ---------- "Bengineer" tip ----------
     * If something goes the wrong way:
     * - If forward moves backward, flip the sign on the distance (distance -> -distance)
     *   or reverse one motor's direction in init().
     * - If right turn goes left, flip the sign of the angle (angle -> -angle).
     */

    @Override
    public void init() {
        // Nothing runs here. This is where you'd normally map hardware:
        //
        // Example (DO NOT UNCOMMENT IN THIS TEACHING FILE unless you want to test):
        //
        // DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        // DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        //
        // // If your left motor physically points reversed, reverse it in software:
        // leftDrive.setDirection(DcMotor.Direction.REVERSE);
        // rightDrive.setDirection(DcMotor.Direction.FORWARD);
        //
        // // Reset encoders before starting any RUN_TO_POSITION commands:
        // leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        // // Use BRAKE so the robot stops more predictably when power = 0:
        // leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        // Telemetry: show driver that init finished:
        // telemetry.addData("TeachMode", "Init complete - read comments");
        // telemetry.update();

        /* Done: no running code in init() for this teaching file. */
    }

    @Override
    public void init_loop() {
        // This loop runs repeatedly before start() if you are in INIT on the driver station.
        // Useful place to show which alliance or options are selected.
        //
        // Example telemetry (commented):
        // telemetry.addData("TeachMode", "Init loop - choose settings and read comments");
        // telemetry.update();
    }

    @Override
    public void start() {
        // Called once when START is pressed on the driver station.
        // Real OpModes might reset timers here or set initial states.
        //
        // Example:
        // runtime.reset();
    }

    @Override
    public void loop() {
        // MAIN TEACHING SECTION: below are explicit, runnable code snippets
        // that perform "drive forward" and "turn right". They are commented
        // out so this file will not move the robot. Copy into your real OpMode
        // and remove comment markers to run them.

        /* -----------------------------------------------------------------
         * CONSTANTS / MATH EXPLANATIONS (copy these into your real program)
         * -----------------------------------------------------------------
         *
         * // Wheel and encoder constants (example values from StarterBot)
         * final double WHEEL_DIAMETER_MM = 96;            // wheel diameter in mm
         * final double ENCODER_TICKS_PER_REV = 537.7;     // ticks per full motor shaft revolution
         * final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
         * final double TRACK_WIDTH_MM = 404;              // distance between left and right wheels (center to center)
         *
         * // Speeds you can tune:
         * final double DRIVE_SPEED = 0.5;   // fraction: 0.0 - 1.0
         * final double ROTATE_SPEED = 0.3;  // slower for turning
         *
         * -----------------------------------------------------------------
         * DRIVE FORWARD EXAMPLE (24 inches)
         * -----------------------------------------------------------------
         *
         * // Step 1: decide how far to move (human-friendly units)
         * double distanceInInches = 24; // positive = forward, negative = backward
         *
         * // Step 2: convert to millimeters if you're using mm constants:
         * double targetPositionTicks = DistanceUnit.INCH.toMm(distanceInInches) * TICKS_PER_MM;
         *
         * // Step 3: set motor target positions (both wheels same sign for straight)
         * leftDrive.setTargetPosition((int) targetPositionTicks);
         * rightDrive.setTargetPosition((int) targetPositionTicks);
         *
         * // Step 4: choose RUN_TO_POSITION mode so the motor controller handles the encoder control
         * leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         * rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         *
         * // Step 5: apply power to start moving (take care: positive vs negative depends on motor directions)
         * leftDrive.setPower(DRIVE_SPEED);
         * rightDrive.setPower(DRIVE_SPEED);
         *
         * // Step 6: wait until motors reach position (or use a timeout). You can poll:
         * while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
         *     // Optional: provide telemetry feedback while moving
         *     telemetry.addData("Left target", leftDrive.getTargetPosition());
         *     telemetry.addData("Left pos", leftDrive.getCurrentPosition());
         *     telemetry.update();
         * }
         *
         * // Step 7: stop the motors and optionally set mode back to RUN_USING_ENCODER
         * leftDrive.setPower(0);
         * rightDrive.setPower(0);
         * leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         * rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         *
         * Bengineer note: If the robot drives backward instead of forward, either:
         * - negate distanceInInches (make it -24), OR
         * - flip one motor's direction in init() with setDirection(...)
         *
         * -----------------------------------------------------------------
         * ROTATE (TURN) EXAMPLE (90 degrees to the RIGHT)
         * -----------------------------------------------------------------
         *
         * // Step 1: decide the angle to rotate (positive = right turn here)
         * double angleDegrees = 90; // positive = right, negative = left
         *
         * // Step 2: compute how many mm the wheel must travel to achieve this rotation
         * //          (angle in radians) * (turning radius)
         * // turning radius for on-the-spot rotation = TRACK_WIDTH_MM / 2
         * double targetMm = AngleUnit.DEGREES.toRadians(angleDegrees) * (TRACK_WIDTH_MM / 2.0);
         *
         * // Step 3: convert mm to encoder ticks
         * double leftTargetTicks  = -(targetMm * TICKS_PER_MM); // left wheel goes backward
         * double rightTargetTicks =  (targetMm * TICKS_PER_MM); // right wheel goes forward
         *
         * // Step 4: set targets and RUN_TO_POSITION mode
         * leftDrive.setTargetPosition((int) leftTargetTicks);
         * rightDrive.setTargetPosition((int) rightTargetTicks);
         *
         * leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         * rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         *
         * // Step 5: apply power (same magnitude on both sides)
         * leftDrive.setPower(ROTATE_SPEED);
         * rightDrive.setPower(ROTATE_SPEED);
         *
         * // Step 6: wait until rotation completes (similar to drive)
         * while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
         *     telemetry.addData("Rotating: left pos/target", leftDrive.getCurrentPosition() + " / " + leftDrive.getTargetPosition());
         *     telemetry.addData("Rotating: right pos/target", rightDrive.getCurrentPosition() + " / " + rightDrive.getTargetPosition());
         *     telemetry.update();
         * }
         *
         * // Step 7: stop and reset or switch modes
         * leftDrive.setPower(0);
         * rightDrive.setPower(0);
         * leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         * rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         *
         * Bengineer tip: If your "right" turn is actually turning left, use -angleDegrees or swap signs
         * for leftTargetTicks/rightTargetTicks.
         *
         * -----------------------------------------------------------------
         * SAFETY AND TESTING ADVICE (READ THIS)
         * -----------------------------------------------------------------
         * - Always test first with very low power (e.g., DRIVE_SPEED = 0.2) to avoid damaging hardware.
         * - Run with the robot elevated (wheels off the ground) to verify wheel directions and encoder
         *   sign before placing the robot on the field.
         * - Use telemetry to print current and target positions during tests so you can debug math.
         * - If positions never change, check that the encoder wires are plugged in to the correct ports.
         *
         * -----------------------------------------------------------------
         * SAMPLE COMBINED SEQUENCE (drive forward then rotate right)
         * -----------------------------------------------------------------
         *
         * // 1) Drive forward 24 inches (copy the DRIVE FORWARD EXAMPLE)
         * // 2) After stopping, call the ROTATE example with angleDegrees = 90
         *
         * // Pseudo-high-level sequence:
         * drive(DRIVE_SPEED, 24, DistanceUnit.INCH, 0.5);   // move forward 24"
         * rotate(ROTATE_SPEED, 90, AngleUnit.DEGREES, 0.5); // turn right 90°
         *
         * Note: drive(...) and rotate(...) are utility methods. You can implement them as in
         * StarterBotAuto or inline the commands shown above.
         */

        // Nothing actually runs in this loop. This file is purely educational.
    }

    @Override
    public void stop() {
        // Nothing to stop in this teaching file.
        // In a real program, always set motor powers to zero here:
        // leftDrive.setPower(0);
        // rightDrive.setPower(0);
    }
}