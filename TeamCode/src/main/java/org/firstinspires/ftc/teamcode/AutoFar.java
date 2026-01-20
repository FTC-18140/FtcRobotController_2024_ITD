package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutoFar")
public class AutoFar extends OpMode {

    DcMotor leftDrive, rightDrive;
    DcMotorEx ShootMotor;//Flywheel
    IMU imu;
    Servo servo1;
    Servo servo2;

    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        ShootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");


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
                new PIDFCoefficients(34.0, 0.007, 0.95, 16)
        );

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

    }

    @Override
    public void start() {
        //What the auto does
        driveForward(0.5, 5500);
        Turn(0.2, 1);
        Shoot(315);
        reload();
        sleep(2000);
        Shoot(315);
        reload();
        sleep(2000);
        Shoot(315);
        reload();
        sleep(2000);
        Turn(0.3, 2000);
        driveBackward(0.5, 3700);
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
    public double  Turn(double power, long timeMs) {
        double target = 90; // turn to 90 degrees
        double heading = imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit());
        double error = target - heading;

        double kP = 0.01; // tune this
        double turnPower = kP * error;

        leftDrive.setPower(turnPower);
        rightDrive.setPower(-turnPower);
        return heading;
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
