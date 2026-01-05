/*
 * Simple autonomous: drive forward then turn right 90 degrees.
 *
 * Assumes robot hardware names: "left_drive", "right_drive".
 * If your robot drives the wrong direction, invert the sign of the distance
 * in drive(...) or of the angle in rotate(...).
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="MoveForwardTurnRightAuto", group="StarterBot")
public class Basic_Movement extends OpMode {

    // Drive / encoder constants (copied from your StarterBotAuto)
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.3;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    // Timers
    private ElapsedTime driveTimer = new ElapsedTime();

    // Motors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // State machine for the simple autonomous
    private enum AutoState {
        MOVE_FORWARD,
        ROTATE_RIGHT,
        COMPLETE
    }
    private AutoState autoState;

    @Override
    public void init() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Match directions to your robot (same as StarterBotAuto)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Info", "Init loop - waiting for start");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start with the first step
        autoState = AutoState.MOVE_FORWARD;
        // Ensure timers are reset
        driveTimer.reset();
    }

    @Override
    public void loop() {
        switch (autoState) {
            case MOVE_FORWARD:
                // Drive forward 24 inches. If this moves backward, change 24 -> -24
                if (drive(DRIVE_SPEED, 24, DistanceUnit.INCH, 0.5)) {
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autoState = AutoState.ROTATE_RIGHT;
                }
                break;

            case ROTATE_RIGHT:
                // Rotate right 90 degrees. If this turns left, change 90 -> -90
                if (rotate(ROTATE_SPEED, 90, AngleUnit.DEGREES, 0.5)) {
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                // Stop motors
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                telemetry.addData("Auto", "Complete");
                break;
        }

        telemetry.addData("State", autoState);
        telemetry.update();
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * Drive a linear distance using RUN_TO_POSITION.
     * @param speed 0-1
     * @param distance in specified distanceUnit (positive = forward in this code)
     * @param distanceUnit unit
     * @param holdSeconds seconds to be within tolerance before reporting success
     * @return true when finished
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftDrive.setTargetPosition((int) targetPosition);
        rightDrive.setTargetPosition((int) targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if(Math.abs(targetPosition - leftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Rotate the robot by a given angle (positive = right turn in this implementation).
     * @param speed 0-1
     * @param angle amount to rotate (AngleUnit-specified)
     * @param angleUnit unit
     * @param holdSeconds seconds to be within tolerance before reporting success
     * @return true when finished
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetMm = angleUnit.toRadians(angle)*(TRACK_WIDTH_MM/2);

        double leftTargetPosition = -(targetMm*TICKS_PER_MM);
        double rightTargetPosition = (targetMm*TICKS_PER_MM);

        leftDrive.setTargetPosition((int) leftTargetPosition);
        rightDrive.setTargetPosition((int) rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if((Math.abs(leftTargetPosition - leftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}