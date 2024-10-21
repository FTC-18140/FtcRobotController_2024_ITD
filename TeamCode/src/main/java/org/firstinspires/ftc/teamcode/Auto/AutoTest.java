package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

@Config
@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThunderBot2024 robot = new ThunderBot2024();
        robot.init(hardwareMap,telemetry);
        robot.drive.pose = new Pose2d(-12,-60,Math.toRadians(90));

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                robot.intake.wristMoveAction(robot.intake.WRIST_MAX),
                robot.intake.spinnerAction(1),
                robot.intake.armMoveAction(0.1)


        ));
        telemetry.update();

    }
}
