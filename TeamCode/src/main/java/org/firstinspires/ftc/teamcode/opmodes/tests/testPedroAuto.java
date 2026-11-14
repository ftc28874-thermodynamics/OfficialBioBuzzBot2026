package org.firstinspires.ftc.teamcode.opmodes.tests;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

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

import org.firstinspires.ftc.teamcode.util.pedroTiles;

@Autonomous(name = "test pedro autonomous")
public class testPedroAuto extends LinearOpMode {

    private Follower follower;

    // Calling pedroTiles methods to get Pose objects for Tile 16, Tile 15, and Tile 4
    private final Pose start = pedroTiles.TILE_16.getPoseWithHeading(90);
    private final Pose path1 = pedroTiles.TILE_15.getPoseWithHeading(180);
    private final Pose point2 = pedroTiles.TILE_4.getPose();

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        assert follower != null;
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
                telemetry.addData("current dist remaining in path: ", follower.distanceToEndpoint());
                telemetry.addData("path # ", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }
}