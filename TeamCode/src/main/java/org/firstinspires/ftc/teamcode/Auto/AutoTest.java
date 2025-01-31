package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

@Config
@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThunderBot2024 robot = new ThunderBot2024();
        robot.init(hardwareMap,telemetry, 0);
        Pose2d startPos = new Pose2d(-15,-60,Math.toRadians(90));
        robot.drive.pose = startPos;

        waitForStart();
        robot.intake.start();

        Actions.runBlocking(
                new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armUpAction(Intake.ARM_MAX_HORIZONTAL)
                )
            )
        );
        telemetry.update();

    }
}
