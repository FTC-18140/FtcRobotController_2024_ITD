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

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        robot.intake.presetAction(Intake.Positions.HIGH_CHAMBER),
                        robot.intake.armUpAction(Intake.Positions.HIGH_CHAMBER.armPos),
                        robot.drive.actionBuilder(startPos)
                                .strafeTo(new Vector2d(-10, -42))
                                .build(),
                        robot.intake.wristMoveAction(0.5),
                        robot.drive.actionBuilder(new Pose2d(-10,-42, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-10, -58))
                                .build(),
                        new SleepAction(1),
                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE)
                )
            )
        );
        telemetry.update();

    }
}
