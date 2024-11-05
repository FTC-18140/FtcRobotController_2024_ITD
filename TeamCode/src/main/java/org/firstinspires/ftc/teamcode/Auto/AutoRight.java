package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

@Config
@Autonomous
public class AutoRight extends LinearOpMode {
    public static Vector2d startPos = new Vector2d(12,-60);
    public static Vector2d basketPos = new Vector2d(-40,-54);
    public static Vector2d samplePos = new Vector2d(50,-40);
    public static Vector2d parkPos = new Vector2d(52,-56);
    @Override
    public void runOpMode() throws InterruptedException {
        //Move to basket () and rotate <-+
        //Lift arm                       |
        //release sample                 |
        //retract arm                    |
        //Move to placed sample ()       |
        //intake sample                  |
        //Repeat-------------------------+
        //park ()

        ThunderBot2024 robot = new ThunderBot2024();

        robot.init(hardwareMap,telemetry);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeTo(basketPos)
                                .build(),
                        robot.intake.wristMoveAction(0.75),
                        new SleepAction(1),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(90)))
                                        .strafeTo(new Vector2d(50,-48))
                                        .strafeTo(samplePos)
                                        .build(),
                                robot.intake.spinnerAction(1)
                        ),
                        robot.intake.armUpAction(16),
                        new SleepAction(2),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0),
                                robot.intake.armDownAction(0),
                                robot.intake.spinnerAction(0),
                                robot.drive.actionBuilder(new Pose2d(samplePos, Math.toRadians(90)))
                                        .strafeTo(new Vector2d(50,-50))
                                        .strafeTo(basketPos)
                                        .build()
                        ),
                        robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(90)))
                                .strafeTo(parkPos)
                                .build()
                )
        ));

    }
}
