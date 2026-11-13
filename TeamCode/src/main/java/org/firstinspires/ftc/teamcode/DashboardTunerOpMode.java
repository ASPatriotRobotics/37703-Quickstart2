package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IMU & PID Dashboard Tuner", group = "Tuning")
public class DashboardTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumImuDrive drive = new MecanumImuDrive(this, hardwareMap);

        telemetry.addLine("FTC Dashboard Tuner Ready!");
        telemetry.addLine("A: Drive Straight | B: Turn to Heading | X: Strafe | Y: Reset Yaw");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.clearBulkCache();

            if (gamepad1.a) {
                drive.driveStraight(MecanumImuDrive.TEST_DISTANCE_INCHES, 0.0, MecanumImuDrive.TEST_MAX_POWER, 4.0);
            } else if (gamepad1.b) {
                drive.turnToHeading(MecanumImuDrive.TEST_TARGET_HEADING, MecanumImuDrive.TEST_MAX_POWER, 3.0);
            } else if (gamepad1.x) {
                drive.strafe(MecanumImuDrive.TEST_STRAFE_INCHES, 0.0, MecanumImuDrive.TEST_MAX_POWER, 4.0);
            } else if (gamepad1.y) {
                drive.resetYaw();
            }

            telemetry.addData("Live Heading (deg)", "%.2f", drive.getHeading());
            telemetry.update();
        }
    }
}