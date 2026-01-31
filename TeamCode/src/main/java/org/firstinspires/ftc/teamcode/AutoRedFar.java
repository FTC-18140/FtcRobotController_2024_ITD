package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="AutoRed_Far")
public class AutoRedFar extends OpMode {
    DcMotor leftDrive, rightDrive;
    DcMotorEx ShootMotor;//Flywheel
    IMU imu;
    Servo servo1;
    Servo servo2;
    WebcamName webcam;

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;

    @Override
    public void init() {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        leftDrive = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        ShootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .build();

        // Match your TeleOp directions
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ShootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(34.0, 0.007, 1.4, 14)
        );

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

    }

    int autoStep = 0;

    boolean isTurning = false;
    double targetAngle = 0;
    double kP = 0.01;

    @Override
    public void start() {
        switch (autoStep) {

            case 0:
                driveForward(0.5, 3000);
                autoStep++;

            case 1:
                Turn(50);
                autoStep++;


            case 2:
                driveForward(0.5,500);
                autoStep++;



            case 3:
                Shoot(375);
                reload();
                autoStep++;


            case 4:
                Shoot(375);
                reload();
                autoStep++;

                break;
        }
    }
    // ---------------- STOP ----------------
    public void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // ---------------- DRIVE FORWARD ----------------
    public void driveForward(double power, long timeMs) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(timeMs);
        stopMotors();
    }

    // ---------------- Shoot ----------------
    public void Shoot(double power) {
        ShootMotor.setVelocity(power);
        /*Doesn't need to be stopped because
        * then it will start- get to velocity - stop and repeat
        * Instead of start - get to velocity - stop when code ends
         */

    }

    // ---------------- LOADING ----------------
    public void reload() {
        servo1.setPosition(0.5);
        servo2.setPosition(0.5);
        sleep(2000);
        servo1.setPosition(0);
        servo2.setPosition(0);
    }

    // ---------------- DRIVE BACKWARD ----------------
    public void driveBackward(double power, long timeMs) {
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(timeMs);
        stopMotors();
    }

    // ---------------- TURN ----------------
    public void Turn(double angle) {
        if (isTurning) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(targetAngle - heading);

            double turnPower = kP * error;
            turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

            leftDrive.setPower(turnPower);
            rightDrive.setPower(-turnPower);

            if (Math.abs(error) < 1.0) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                isTurning = false;
            }
        }

    }

    public void loop(){

        //Information
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("power", leftDrive.getPower());
        packet.put("power", rightDrive.getPower());
        packet.put("ShootMotor rpm", ShootMotor.getVelocity());
        packet.put("ShootMotor PID", ShootMotor.getPIDFCoefficients(ShootMotor.getMode()));
        packet.put("Servo1 position", servo1.getPosition());
        packet.put("Servo2 position", servo2.getPosition());
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("ShootMotor rpm:", ShootMotor.getVelocity());
        telemetry.addData("PID Coefficients:", ShootMotor.getPIDFCoefficients(ShootMotor.getMode()));
        telemetry.addData("Servo1 position:", servo1.getPosition());
        telemetry.addData("Servo2 position:", servo2.getPosition());
        telemetry.addData("Left motor power:", leftDrive.getPower());
        telemetry.addData("Right motor power:", rightDrive.getPower());
        telemetry.update();
    }
}


