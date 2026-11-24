package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath1", group = "Autonomous")
public class AutoPath1 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(0.4598, 2.749, -174.5991);
    private final Pose point2 = poseFactory.of(56.5595, 137.5627, 67.4064);
    private final Pose point3Start = poseFactory.of(56.5595, 137.5627, 0);
    private final Pose point3 = poseFactory.of(47.1432, 135.9283, 0);
    private final Pose point4 = poseFactory.of(56.6625, 136.857, -174.428);
    private final Pose point5 = poseFactory.of(5.1063, 46.8687, 60.1907);
    private final Pose point6 = poseFactory.of(54.8461, 7.641, 141.7386);
    private final Pose point7 = poseFactory.of(3.5198, 97.988, -60.399);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).tangent();
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).reverseTangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }
}
