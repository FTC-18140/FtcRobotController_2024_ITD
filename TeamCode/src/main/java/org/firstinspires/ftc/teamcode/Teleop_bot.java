package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop_bot extends OpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotorEx ShootMotor;
    Servo servo1;
    Servo servo2;
    IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        leftDrive = hardwareMap.get(DcMotor.class, "leftMotor");
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setDirection(Servo.Direction.FORWARD);
        //servo1.setPosition(0.0);
        servo2.setDirection(Servo.Direction.FORWARD);
        //servo2.setPosition(0.0);
        //telemetry.addData("servo1 postion", servo1.getPosition());
        //telemetry.addData("servo2 postion", servo2.getPosition());
        ShootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ShootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(36.0, 0.007, 0.9, 16)
        );
        IMU.Parameters parameters = new IMU.Parameters
                (new RevHubOrientationOnRobot
                        (RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
                imu.initialize(parameters);
                imu.resetYaw();
        telemetry.addData("Status", "Initalized");
    }
    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    @Override
    public void loop() {
        // Basic arcade drive
        double drive = gamepad1.left_stick_y;  // forward/back
        double turn = gamepad1.right_stick_x;   // left/right

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Speed scaling
        double speedScale = 0.7;   // default normal speed

        if (gamepad1.left_bumper) {
            speedScale = 1;// fast mode (30% faster)
            telemetry.addLine("Fast mode");

        } else if (gamepad1.right_bumper) {
            speedScale = 0.4;// slow mode
            telemetry.addLine("Slow mode");

        }

        // Apply scaling
        leftPower  *= speedScale;
        rightPower *= speedScale;
        // Send to motors
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        //ATTACHMENTS

        //organizer forwards
        if (gamepad1.circle) {
            servo1.setDirection(Servo.Direction.FORWARD);
            servo2.setDirection(Servo.Direction.REVERSE);
            servo1.setPosition(1);
            servo2.setPosition(1);
        }else {
            servo1.setPosition(0.0);
            servo2.setPosition(0.0);
        }
        //organizer backwards
        if (gamepad1.square) {
            servo1.setPosition(0);
            servo2.setPosition(0);
        }
        //Shooting
        if (gamepad1.right_trigger > 0.9) {
            ShootMotor.setVelocity(320);

        }else {
            ShootMotor.setVelocity(0);


        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("power", leftDrive.getPower());
        packet.put("power", rightDrive.getPower());
        packet.put("ShootMotor rpm", ShootMotor.getVelocity());
        packet.put("ShootMotor PID", ShootMotor.getPIDFCoefficients(ShootMotor.getMode()));
        packet.put("Servo1 position", servo1.getPosition());
        packet.put("Servo2 position", servo2.getPosition());
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Velocity:", ShootMotor.getVelocity());
        telemetry.addData("Left wheel speed", leftDrive.getPower());
        telemetry.addData("Right wheel speed:", rightDrive.getPower());
        telemetry.addData("servo1 postion", servo1.getPosition());
        telemetry.addData("servo2 postion", servo2.getPosition());
        telemetry.update();
        }
    }

/*
        //Chassis

        //Forward
        if (gamepad1.right_stick_y > 0.9) {
            leftDrive.setPower(0.7);
            rightDrive.setPower(0.7);
        } else if (gamepad1.right_stick_x < 0.9){
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        //Backwards
        if (-gamepad1.right_stick_y > 0.9) {
            leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDrive.setPower(0.7);
            rightDrive.setPower(0.7);
        }else if (- gamepad1.right_stick_y < 0.9){
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        //Turn left
        if (gamepad1.right_stick_x > 0.9) {
            leftDrive.setPower(1);
            rightDrive.setPower(-1);
        }
        //Turn right
         else if (-gamepad1.right_stick_x > 0.9) {
            rightDrive.setPower(1);
            leftDrive.setPower(-1);
        }else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        //Fast mode
        if (gamepad1.right_trigger > 0.9){
            rightDrive.setPower(1);
            leftDrive.setPower(1);
        }
        else if (gamepad1.right_trigger < 0.9){
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
        //Slow Mode
        if (gamepad1.left_trigger > 0.9){
            rightDrive.setPower(0.4);
            leftDrive.setPower(0.4);
        }else if (gamepad1.left_trigger < 0.9){
            rightDrive.setPower(0);
            leftDrive.setPower(0);
        }

 */



