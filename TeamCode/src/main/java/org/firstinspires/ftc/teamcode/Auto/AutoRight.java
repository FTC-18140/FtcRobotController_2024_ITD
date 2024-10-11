package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

@Config
@Autonomous
public class AutoRight extends LinearOpMode {
    public static Vector2d startPos = new Vector2d(12,-60);
    public static Vector2d basketPos = new Vector2d(-50,-50);
    public static Vector2d samplePos = new Vector2d(-48,-36);
    public static Vector2d parkPos = new Vector2d(-56,-56);
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

        Action moveToBasket1;
        Action moveToSample;
        Action moveToBasket2;
        Action park;
        robot.init(hardwareMap,telemetry);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));

        moveToBasket1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeTo(new Vector2d(-48,-48))
                .turn(Math.toRadians(135))
                .build();
        moveToSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeTo(new Vector2d(-48,-48))
                .turn(Math.toRadians(135))
                .build();
        moveToBasket2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeTo(new Vector2d(-48,-48))
                .turn(Math.toRadians(135))
                .build();
        park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeTo(new Vector2d(-48,-48))
                .turn(Math.toRadians(135))
                .build();
        waitForStart();

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                .strafeTo(basketPos)
                .turn(Math.toRadians(-45))
                        .waitSeconds(2)
                .build()
        );
        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .turn(Math.toRadians(45))
                        .strafeTo(samplePos)
                        .waitSeconds(2)
                        .build()
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeTo(basketPos)
                        .turn(Math.toRadians(-45))
                        .waitSeconds(2)
                        .build()
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .turn(Math.toRadians(45))
                        .strafeTo(parkPos)
                        .build()
        );
    }
}
