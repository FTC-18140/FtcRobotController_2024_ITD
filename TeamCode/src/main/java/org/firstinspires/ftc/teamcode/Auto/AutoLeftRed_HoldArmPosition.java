package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

@Config
@Autonomous
@Disabled
public class AutoLeftRed_HoldArmPosition extends LinearOpMode {
    public static Vector2d startPos = new Vector2d(-15,-60);
    public static Vector2d basketPos = new Vector2d(-53.5,-53);
    public static Vector2d samplePos = new Vector2d(-49,-38);
    public static Vector2d parkPos = new Vector2d(-26,-11);
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

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                robot.lift.liftToAction(2200),
                new SequentialAction(
                        new ParallelAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeTo(new Vector2d(startPos.x,basketPos.y))
                                        .strafeToLinearHeading(basketPos, Math.toRadians(45))
                                        .build(),
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(28)
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(1)
                        )
                        ,robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.intake.spinnerAction(0),
                                robot.intake.elbowAction(0.5),
                                robot.drive.actionBuilder(new Pose2d(basketPos.x, basketPos.y, Math.toRadians(45)))
                                        .turn(Math.toRadians(45))
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.73),
                                robot.intake.spinnerAction(1),
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(90)))
                                        .strafeTo(new Vector2d(samplePos.x, -48))
                                        .strafeTo(samplePos)
                                        .build()
                        ),
                        new SleepAction(1),
                        new ParallelAction(
                                robot.intake.armUpAction(20),
                                robot.intake.checkForSample("yellowred", 5)
                        ),
                        new SleepAction(1),
                        new ParallelAction(
                                robot.intake.spinnerAction(0),
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(28),
                                robot.drive.actionBuilder(new Pose2d(samplePos, Math.toRadians(90)))
                                        .turn(Math.toRadians(-45))
                                        .strafeTo(basketPos)
                                        .build()
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(1)
                        )
                        ,
                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        robot.intake.spinnerAction(0),
                        robot.intake.wristMoveAction(0),
                        new SleepAction(0.75),
                        robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(-42, -11))
                                .turn(Math.toRadians(90))
                                .build(),
                        robot.drive.actionBuilder(new Pose2d(new Vector2d(-42, -11), Math.toRadians(180)))
                                .strafeTo(parkPos)
                                .build()
                        )
                )
        );
    }
}
