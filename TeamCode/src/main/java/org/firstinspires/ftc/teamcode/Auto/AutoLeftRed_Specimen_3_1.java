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

import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

@Config
@Autonomous
public class AutoLeftRed_Specimen_3_1 extends LinearOpMode {
    public static Vector2d startPos = AutoPositions.Positions.START_LEFT.position;
    public static Vector2d basketPos = AutoPositions.Positions.BASKET.position;
    public static Vector2d basketPosStart = new Vector2d(-54, -53);
    public static Vector2d samplePos_1 = AutoPositions.Positions.SAMPLE_1_LEFT.position;
    public static Vector2d samplePos_2 = new Vector2d(-59.5, -38);
    public static Vector2d samplePos_3 = new Vector2d(-55.5, -40);
    public static Vector2d parkPos = AutoPositions.Positions.ASCENT_ZONE.position;
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

        robot.init(hardwareMap,telemetry, 0);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));

        waitForStart();
        robot.intake.start();

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        //Score Preset Specimen

                        robot.intake.presetAction(Intake.Positions.HIGH_CHAMBER),
                        robot.intake.armUpAction(Intake.Positions.HIGH_CHAMBER.armPos),
                        robot.drive.actionBuilder(new Pose2d(startPos, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-10, -42))
                                .build(),
                        robot.intake.wristMoveAction(0.5),
                        robot.drive.actionBuilder(new Pose2d(-10,-42, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-10, -58))
                                .strafeTo(basketPosStart)
                                .build()

                        //First Cycle

                        ,robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.81),
                                robot.intake.spinnerAction(1),
                                robot.drive.actionBuilder(new Pose2d(basketPosStart, Math.toRadians(45)))
                                        .strafeToSplineHeading(new Vector2d(samplePos_1.x, -48), Math.toRadians(90))
                                        .strafeTo(samplePos_1)
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.armUpAction(30),
                                robot.intake.checkForSample("yellowred", 7)
                        ),
                        new ParallelAction(
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(Intake.Positions.HIGH_BASKET.armPos),
                                robot.drive.actionBuilder(new Pose2d(samplePos_1, Math.toRadians(90)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build()
                        ),
                        new SleepAction(0.5),
                        robot.intake.wristMoveAction(0.25),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(0.5)
                        ),
                        //Second Cycle

                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.83),
                                robot.intake.spinnerAction(1),
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                        .strafeToSplineHeading(new Vector2d(samplePos_2.x, -48), Math.toRadians(90))
                                        .strafeTo(samplePos_2)
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.armUpAction(30),
                                robot.intake.checkForSample("yellowred", 7)
                        ),
                        new ParallelAction(
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(Intake.Positions.HIGH_BASKET.armPos),
                                robot.drive.actionBuilder(new Pose2d(samplePos_2, Math.toRadians(90)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build()
                        ),
                        new SleepAction(0.5),
                        robot.intake.wristMoveAction(0.25),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(0.5)
                        ),
                        //Third Cycle

                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.83),
                                robot.intake.spinnerAction(1),
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                        .strafeToSplineHeading(new Vector2d(samplePos_3.x, -50), Math.toRadians(135))
                                        .strafeTo(samplePos_3)
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.armUpAction(20),
                                robot.intake.checkForSample("yellowred", 10)
                        ),
                        new SleepAction(1),
                        new ParallelAction(
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(Intake.Positions.HIGH_BASKET.armPos),
                                robot.drive.actionBuilder(new Pose2d(samplePos_3, Math.toRadians(135)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build()
                        ),
                        new SleepAction(0.5),
                        robot.intake.wristMoveAction(0.25),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(0.5)
                        ),
                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        robot.intake.spinnerAction(0),
                        robot.intake.wristMoveAction(0)
                    )
                )
        );
    }
}
