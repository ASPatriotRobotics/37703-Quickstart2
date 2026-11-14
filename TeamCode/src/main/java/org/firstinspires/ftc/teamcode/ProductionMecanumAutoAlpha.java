package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name = "Production Mecanum Auto Alpha", group = "Competition")
public class ProductionMecanumAutoAlpha extends LinearOpMode {


    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    public final int LAUNCHER_TARGET_VELOCITY = 1440; //2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 1400; //2571 RPM

    double intakePower=0.5;

    @Override
    public void runOpMode() {
        // Initialize drivetrain controller with tuned constants
        MecanumImuDrive drive = new MecanumImuDrive(this, hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

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


        telemetry.addLine("Robot Calibrated & Ready.");
        telemetry.update();

        // Lock in 0 degrees based on exactly how you tile-aligned the robot during setup
        waitForStart();
        drive.resetYaw();

        if (opModeIsActive()) {
            // STEP 1: Drive to Score Location (e.g., forward 32 inches)
            drive.driveStraight(32.0, 0.0, 0.7, 3.0);

            // STEP 2: Turn to face the scoring element (e.g., turn 45 degrees left)
            drive.turnToHeading(90.0, 0.5, 2.0);

            // STEP 3: Actuate mechanisms to score (Put your servo/motor actions here)
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            sleep(2000);
            windmillServo.setPower(-1);
            intake.setPower(intakePower);
            sleep(5000);
            launcher.setVelocity(0);
            windmillServo.setPower(0);
            intake.setPower(0);


            // STEP 4: Back away from the score position cleanly holding the 45-degree angle
            drive.driveStraight(-72.0, -90.0, 0.5, 2.0);

            // STEP 5: Turn toward the human player wall or park zone (e.g., face -90 degrees)
            drive.turnToHeading(0.0, 0.6, 2.5);

            // STEP 6: Strafe safely into the designated parking zone
            drive.strafe(32.0, 0.0, 0.6, 3.0);
        }
    }
}