package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Production Mecanum Auto Alpha", group = "Competition")
public class ProductionMecanumAutoAlpha extends LinearOpMode {

    public final int LAUNCHER_TARGET_VELOCITY = 1140; //2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 1070; //2571 RPM

    double intakePower = 0.5;

    @Override
    public void runOpMode() {
        // Merge Driver Station telemetry with FTC Dashboard graphing output
        Telemetry customTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize drivetrain controller with tuned constants
        MecanumImuDrive drive = new MecanumImuDrive(this, hardwareMap);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        CRServo leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        CRServo rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        customTelemetry.addLine("Robot Calibrated & Ready.");
        customTelemetry.update();

        boolean isRedAlliance = true;
        boolean isParkOnly = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isRedAlliance = false; // Blue
            if (gamepad1.b) isRedAlliance = true;  // Red
            if (gamepad1.a) isParkOnly = !isParkOnly;

            customTelemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            customTelemetry.addData("Strategy", isParkOnly ? "PARK ONLY" : "SCORE + PARK");
            customTelemetry.update();
        }

        // Lock in 0 degrees based on exactly how you tile-aligned the robot during setup
        waitForStart();
        drive.resetYaw();

        if (opModeIsActive()) {
            if (!isParkOnly) {
                // STEP 1: Drive to Score Location (e.g., forward 32 inches)
                drive.driveStraight(44.0, 0.0, 0.7, 3.0);

                // STEP 2: Turn to face the scoring element (e.g., turn 90 degrees left)
                drive.turnToHeading(-90.0, 0.5, 2.0);
                drive.driveStraight(4.0, -90, 0.5, 1.0);
                // STEP 3: Actuate mechanisms to score with Live Telemetry Tracking
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Active Loop for Spin-up (Replaces blind sleep)
                ElapsedTime timer = new ElapsedTime();
                while (opModeIsActive() && timer.seconds() < 2.0) {
                    drive.clearBulkCache(); // Ensure fresh encoder readings
                    double currentVelocity = launcher.getVelocity();

                    customTelemetry.addLine("=== LAUNCHER SPIN UP ===");
                    customTelemetry.addData("Target Velocity", LAUNCHER_TARGET_VELOCITY);
                    customTelemetry.addData("Current Velocity", currentVelocity);
                    customTelemetry.addData("Velocity Error", LAUNCHER_TARGET_VELOCITY - currentVelocity);
                    customTelemetry.update();
                }

                // Engage feeding mechanisms
                windmillServo.setPower(-1);
                intake.setPower(intakePower);

                // Active Loop for Shooting Sequence (Replaces blind sleep)
                timer.reset();
                while (opModeIsActive() && timer.seconds() < 5.0) {
                    drive.clearBulkCache();
                    double currentVelocity = launcher.getVelocity();

                    customTelemetry.addLine("=== FIRING SEQUENCER ===");
                    customTelemetry.addData("Target Velocity", LAUNCHER_TARGET_VELOCITY);
                    customTelemetry.addData("Current Velocity", currentVelocity);
                    customTelemetry.addData("Stable Status", currentVelocity >= LAUNCHER_MIN_VELOCITY ? "READY TO FIRE" : "RECOVERING RPM");
                    customTelemetry.update();
                }

                // Shutdown scoring components
                launcher.setVelocity(0);
                windmillServo.setPower(0);
                intake.setPower(0);

                // STEP 4: Back away from the score position cleanly holding the 90-degree angle
                drive.driveStraight(-100.0, -93.0, 0.7, 7.0);

                // STEP 5: Turn toward the human player wall or park zone (e.g., face -90 degrees)
                // drive.turnToHeading(0.0, 0.6, 2.5);

                // STEP 6: Strafe safely into the designated parking zone holding 90.0
                drive.strafe(32.0, -90.0, 0.6, 3.0);
            } else {
                drive.driveStraight(3.0, 0.0, 0.5, 2.0);
                drive.turnToHeading(-90.0, 0.5, 2.0);
                drive.driveStraight(5.0, -90.0, 0.6, 2.0);
            }
        }
    }
}
