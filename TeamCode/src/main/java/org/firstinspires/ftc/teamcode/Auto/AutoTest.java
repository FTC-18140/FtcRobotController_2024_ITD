package org.firstinspires.ftc.teamcode.Auto;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.StraferBot;

@Config
@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        StraferBot robot = new StraferBot();
        Action park;
        robot.init(hardwareMap,telemetry);
        robot.drive.pose = new Pose2d(-12,-60,Math.toRadians(90));

        park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeTo(new Vector2d(-48,-48))
                .turn(Math.toRadians(135))
                .build();
        waitForStart();

        Actions.runBlocking(park);
    }
}
