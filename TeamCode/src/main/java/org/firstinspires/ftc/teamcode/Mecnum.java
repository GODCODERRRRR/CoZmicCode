package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GoBilda TeleOp (Stable Launcher)", group = "Linear OpMode")
public class Mecnum extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotor backLeft;
    private DcMotor topLeft;
    private DcMotor backRight;
    private DcMotor topRight;

    // --- Launcher Motor ---
    private DcMotorEx launchingMotor;

    // --- Servos ---
    private Servo leftMover;
    private Servo rightMover;

    // --- Launcher Constants ---
    public static final double TICKS_PER_REV = 28;
    public static final double GEAR_RATIO = 3.7;
    public static final double TARGET_RPM = 999;
    public static final double TARGET_VELOCITY = (TARGET_RPM / 60.0) * TICKS_PER_REV * GEAR_RATIO;

    // --- Servo Constants ---
    public static final double SERVO_HOME = 0.5;
    public static final double SERVO_PUSH = 0.0;
    public static final double SERVO_REVERSE = 1.0;

    // --- PIDF Tune (for GoBilda Yellow Jacket motor) ---
    // Adjust if needed — these are stable baseline values.
    private static final PIDFCoefficients LAUNCH_PIDF = new PIDFCoefficients(30.0, 0.0, 12.0, 12.0);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- Hardware Mapping ---
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        launchingMotor = hardwareMap.get(DcMotorEx.class, "Launching");
        leftMover = hardwareMap.get(Servo.class, "leftMover");
        rightMover = hardwareMap.get(Servo.class, "rightMover");

        // --- Drivetrain Directions ---
        // This is a common configuration for Mecanum drive.
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Brake mode for precision ---
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Launcher Setup ---
        launchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCH_PIDF);

        // --- Servo Setup ---
        leftMover.setPosition(SERVO_HOME);
        rightMover.setPosition(SERVO_HOME);

        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        long launchSpinUpTime = 0;
        boolean launcherOn = false;

        while (opModeIsActive()) {

            // --- Mecanum Drive ---
            // The y-axis is inverted on the gamepad
            double y = -gamepad1.left_stick_y;
            // Strafing can sometimes be imperfect, this helps counteract it
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all powers are scaled down proportionally if the sum is > 1
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Corrected formulas for the given motor directions
            // With left motors reversed, the signs for y and rx need to be flipped
            double topLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double topRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            topLeft.setPower(topLeftPower);
            backLeft.setPower(backLeftPower);
            topRight.setPower(topRightPower);
            backRight.setPower(backRightPower);

            // --- Launcher Control ---
            if (gamepad1.a && !launcherOn) {
                // Turn on and allow spin-up delay
                launchingMotor.setVelocity(TARGET_VELOCITY);
                launcherOn = true;
                launchSpinUpTime = System.currentTimeMillis();
            }

            if (gamepad1.x) {
                // Stop launcher
                launchingMotor.setVelocity(0);
                launcherOn = false;
            }

            // --- Servo Control ---
            if (gamepad1.b) {
                // Only feed balls after spin-up delay (e.g., 1.2 sec)
                if (launcherOn && (System.currentTimeMillis() - launchSpinUpTime) > 1200) {
                    leftMover.setPosition(SERVO_PUSH);
                    rightMover.setPosition(SERVO_PUSH);
                }
            } else if (gamepad1.y) {
                leftMover.setPosition(SERVO_REVERSE);
                rightMover.setPosition(SERVO_REVERSE);
            } else {
                leftMover.setPosition(SERVO_HOME);
                rightMover.setPosition(SERVO_HOME);
            }

            // --- Telemetry for tuning ---
            double currentVelocity = launchingMotor.getVelocity();
            double currentRPM = (currentVelocity / TICKS_PER_REV / GEAR_RATIO) * 60;
            double error = TARGET_RPM - currentRPM;

            telemetry.addData("Drive", "Y: %.2f | X: %.2f | RX: %.2f", y, x, rx);
            telemetry.addData("Launcher", "Target: %.0f RPM | Current: %.0f RPM | Error: %.0f", TARGET_RPM, currentRPM, error);
            telemetry.addData("Launcher PIDF", "P:%.1f I:%.1f D:%.1f F:%.1f", LAUNCH_PIDF.p, LAUNCH_PIDF.i, LAUNCH_PIDF.d, LAUNCH_PIDF.f);
            telemetry.update();
        }

        // --- Stop All Motors ---
        topLeft.setPower(0);
        backLeft.setPower(0);
        topRight.setPower(0);
        backRight.setPower(0);
        launchingMotor.setPower(0);
    }
}