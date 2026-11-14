package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Config
public class MecanumImuDrive {
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;
    private final VoltageSensor batteryVoltageSensor;
    private final List<LynxModule> allHubs;

    // --- LIVE-TUNABLE DASHBOARD CONSTANTS ---
    // goBILDA 312 RPM (537.7 PPR) + 96mm Mecanum wheels (1:1 ratio) = 45.2848 ticks/inch
    public static double TICKS_PER_INCH = 41.80;
    // Strafing experiences roller slip; tune this empirically over a 36" test strafe
    public static double STRAFE_TICKS_PER_INCH = 46.20;

    // Turn & Heading Hold PID Constants
    public static double kP_TURN = 0.024;
    public static double kD_TURN = 0.0025;
    public static double kP_HEADING_HOLD = 0.030;

    // Friction & Voltage Compensation
    public static double MIN_POWER = 0.16;
    public static double NOMINAL_VOLTAGE = 12.0;

    // Dashboard Testing Variables (Used by the tuning OpMode below)
    public static double TEST_DISTANCE_INCHES = 24.0;
    public static double TEST_STRAFE_INCHES = 24.0;
    public static double TEST_TARGET_HEADING = 90.0;
    public static double TEST_MAX_POWER = 0.65;

    public MecanumImuDrive(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;

        // Combine Driver Station telemetry with FTC Dashboard telemetry
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Enable Manual Bulk Caching for ultra-fast loop times (< 10ms)
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize Motors (Ensure these 4 names match your Robot Configuration)
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        // Standard Mecanum reversal (Left side reversed)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();

        // Initialize Universal IMU: Logo facing LEFT, USB ports facing UP
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void driveStraight(double inches, double targetHeading, double maxPower, double timeoutSec) {
        resetEncoders();
        double targetTicks = Math.abs(inches) * TICKS_PER_INCH;
        double direction = Math.signum(inches);
        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec) {
            clearBulkCache();

            double avgTicks = (Math.abs(frontLeft.getCurrentPosition()) +
                    Math.abs(frontRight.getCurrentPosition()) +
                    Math.abs(backLeft.getCurrentPosition()) +
                    Math.abs(backRight.getCurrentPosition())) / 4.0;

            double remainingTicks = targetTicks - avgTicks;
            if (remainingTicks <= 15) break;

            // Smooth deceleration in the last 10 inches to prevent wheel slip
            double rampFactor = Range.clip(remainingTicks / (10.0 * TICKS_PER_INCH), 0.0, 1.0);
            double driveSpeed = direction * Math.max(MIN_POWER, maxPower * rampFactor);

            double currentHeading = getHeading();
            double headingError = AngleUnit.normalizeDegrees(targetHeading - currentHeading);
            double turnCorrection = headingError * kP_HEADING_HOLD;

            setMecanumPower(driveSpeed, 0, turnCorrection);

            // Stream live graphing data to FTC Dashboard
            telemetry.addData("Target Inches", inches);
            telemetry.addData("Current Inches", (avgTicks / TICKS_PER_INCH) * direction);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.update();
        }
        stop();
    }

    public void strafe(double inches, double targetHeading, double maxPower, double timeoutSec) {
        resetEncoders();
        double targetTicks = Math.abs(inches) * STRAFE_TICKS_PER_INCH;
        double direction = Math.signum(inches); // + = Left, - = Right
        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec) {
            clearBulkCache();

            double avgTicks = (Math.abs(frontLeft.getCurrentPosition()) +
                    Math.abs(frontRight.getCurrentPosition()) +
                    Math.abs(backLeft.getCurrentPosition()) +
                    Math.abs(backRight.getCurrentPosition())) / 4.0;

            double remainingTicks = targetTicks - avgTicks;
            if (remainingTicks <= 15) break;

            double rampFactor = Range.clip(remainingTicks / (8.0 * STRAFE_TICKS_PER_INCH), 0.0, 1.0);
            double strafeSpeed = direction * Math.max(MIN_POWER + 0.04, maxPower * rampFactor);

            double currentHeading = getHeading();
            double headingError = AngleUnit.normalizeDegrees(targetHeading - currentHeading);
            double turnCorrection = headingError * kP_HEADING_HOLD;

            setMecanumPower(0, strafeSpeed, turnCorrection);

            telemetry.addData("Target Strafe", inches);
            telemetry.addData("Current Strafe", (avgTicks / STRAFE_TICKS_PER_INCH) * direction);
            telemetry.addData("Heading Error", headingError);
            telemetry.update();
        }
        stop();
    }

    public void turnToHeading(double targetHeading, double maxPower, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        double lastError = AngleUnit.normalizeDegrees(targetHeading - getHeading());
        double lastTime = timer.seconds();

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec) {
            clearBulkCache();
            double currentTime = timer.seconds();
            double dt = Math.max(0.001, currentTime - lastTime);

            double currentHeading = getHeading();
            double error = AngleUnit.normalizeDegrees(targetHeading - currentHeading);
            double derivative = (error - lastError) / dt;

            // Settle within 1.0 degree for 150ms
            if (Math.abs(error) < 1.0) {
                if (settleTimer.milliseconds() > 150) break;
            } else {
                settleTimer.reset();
            }

            double output = (error * kP_TURN) + (derivative * kD_TURN);
            output = Range.clip(output, -maxPower, maxPower);

            if (Math.abs(error) >= 1.0 && Math.abs(output) < MIN_POWER) {
                output = Math.signum(output) * MIN_POWER;
            }

            setMecanumPower(0, 0, output);

            // Graph Target vs Current Heading in FTC Dashboard!
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Turn Error", error);
            telemetry.update();

            lastError = error;
            lastTime = currentTime;
        }
        stop();
    }

    public void setMecanumPower(double forward, double strafe, double turn) {
        double fl = forward - strafe - turn;
        double fr = forward + strafe + turn;
        double bl = forward + strafe - turn;
        double br = forward - strafe + turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        double voltageScale = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        frontLeft.setPower((fl / max) * voltageScale);
        frontRight.setPower((fr / max) * voltageScale);
        backLeft.setPower((bl / max) * voltageScale);
        backRight.setPower((br / max) * voltageScale);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }
}