package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Back2LaunchRight90Forward2Auto", group="StarterBot")
public class AutoRedLarge extends OpMode {

    // Feeder timing
    final double FEED_TIME = 0.20; // seconds feeder runs per shot
    final double TIME_BETWEEN_SHOTS = 2.0; // seconds between shots

    // Launcher speeds
    final double LAUNCHER_TARGET_VELOCITY = 1350;
    final double LAUNCHER_MIN_VELOCITY = 1300;

    // Drive / encoder constants
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.3;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    // Timers
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();

    // Motors / servos
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Launcher state machine
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }
    private LaunchState launchState;

    // Autonomous state machine
    private enum AutoState {
        MOVE_BACK_2FT,
        LAUNCH_SEQUENCE,
        ROTATE_RIGHT_90,
        FORWARD_2FT,
        COMPLETE
    }
    private AutoState autoState;

    // Shots to fire
    private int shotsToFire = 3;

    @Override
    public void init() {
        // initial states
        autoState = AutoState.MOVE_BACK_2FT;
        launchState = LaunchState.IDLE;

        // hardware map
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // directions (adjust if needed for your robot)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // brake behavior
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // launcher config
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(300, 0, 0, 10));
        } catch (Exception e) {
            // ignore if unsupported on some hardware modules
        }

        // feeder directions
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // ensure feeders idle
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addData("Waiting", "Press START to run auto");
        telemetry.update();
    }

    @Override
    public void start() {
        // reset timers
        driveTimer.reset();
        shotTimer.reset();
        feederTimer.reset();
    }

    @Override
    public void loop() {
        switch (autoState) {
            case MOVE_BACK_2FT:
                // Move backward 2 ft = -24 inches
                if (drive(DRIVE_SPEED, -24, DistanceUnit.INCH, 0.5)) {
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    // prepare to shoot
                    shotsToFire = 3;
                    autoState = AutoState.LAUNCH_SEQUENCE;
                }
                break;

            case LAUNCH_SEQUENCE:
                // Request a shot when shots remain
                if (launch(shotsToFire > 0)) {
                    shotsToFire -= 1;
                    if (shotsToFire <= 0) {
                        // done shooting
                        launcher.setVelocity(0);
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);
                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        autoState = AutoState.ROTATE_RIGHT_90;
                    }
                }
                break;

            case ROTATE_RIGHT_90:
                // Right 90 degrees -> positive angle in this implementation (positive = right)
                if (rotate(ROTATE_SPEED, 90, AngleUnit.DEGREES, 0.5)) {
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autoState = AutoState.FORWARD_2FT;
                }
                break;

            case FORWARD_2FT:
                if (drive(DRIVE_SPEED, 24, DistanceUnit.INCH, 0.5)) {
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                launcher.setVelocity(0);
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                telemetry.addData("Auto", "Complete");
                break;
        }

        // Telemetry
        telemetry.addData("AutoState", autoState);
        telemetry.addData("LaunchState", launchState);
        telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                leftDrive.getTargetPosition(), rightDrive.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        launcher.setVelocity(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    /**
     * Launches one ball. When a shot is requested (shotRequested == true) the method will begin
     * spinning up the launcher and then feed when velocity threshold is reached.
     * Returns true for one cycle after a ball has been launched.
     */
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY){
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
        }
        return false;
    }

    /**
     * Drive a linear distance using RUN_TO_POSITION.
     * @param speed 0-1
     * @param distance in specified distanceUnit (positive = forward)
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

        if (Math.abs(targetPosition - leftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Rotate the robot by a given angle (positive = right turn).
     * @param speed 0-1
     * @param angle amount to rotate (AngleUnit-specified)
     * @param angleUnit unit
     * @param holdSeconds seconds to be within tolerance before reporting success
     * @return true when finished
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = (targetMm * TICKS_PER_MM);

        leftDrive.setTargetPosition((int) leftTargetPosition);
        rightDrive.setTargetPosition((int) rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if ((Math.abs(leftTargetPosition - leftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}