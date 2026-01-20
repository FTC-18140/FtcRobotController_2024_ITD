package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

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

@Autonomous(name="AutoClose")
public class AutoClose extends OpMode {

    DcMotor leftDrive, rightDrive;
    DcMotorEx ShootMotor;//flywheel
    IMU imu;
    Servo servo1;
    Servo servo2;

    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        ShootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");

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
                new PIDFCoefficients(34.0, 0.007, 0.9, 16)
        );

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //was going to use encoders decide to use time based//Feel free to delete all th imu stuff
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
    }

    // ---------------- STOP ----------------
    public void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
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

    // ---------------- TURN ----------------
    public void Turn(double power, long timeMs) {
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(timeMs);
        stopMotors();
    }

        // ---------------- DRIVE BACKWARD ----------------

        public void driveBackward(double power, long timeMs){
            leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDrive.setPower(power);
            rightDrive.setPower(power);
            sleep(timeMs);
            stopMotors();
        }
        public void start(){
        //What the auto does
            Shoot(315);
            reload();
            sleep(2000);
            Shoot(315);
            reload();
            sleep(2000);
            Shoot(315);
            reload();
            sleep(2000);
        }
        public void loop(){
        //Information
        telemetry.addData("ShootMotor rpm:", ShootMotor.getVelocity());
        telemetry.addData("PID Coefficients:", ShootMotor.getPIDFCoefficients(ShootMotor.getMode()));
        telemetry.addData("Servo1 position:", servo1.getPosition());
        telemetry.addData("Servo2 position:", servo2.getPosition());
        telemetry.addData("Left motor power:", leftDrive.getPower());
        telemetry.addData("Right motor power:", rightDrive.getPower());
    }

}