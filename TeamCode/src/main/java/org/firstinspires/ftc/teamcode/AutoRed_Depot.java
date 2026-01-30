package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="AutoRed_Depot")
public class AutoRed_Depot extends OpMode {

    DcMotor leftDrive, rightDrive;
    DcMotorEx ShootMotor;//flywheel
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
                new PIDFCoefficients(34.0, 0.007, 1.4, 14)
        );

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    int autoStep = 0;

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
        public void start(){
        switch (autoStep){

            case 0:
                Shoot(315);
                reload();
                sleep(2000);
                telemetry.addLine("case 0 finished");
                telemetry.update();
                autoStep++;

            case 1:
                Shoot(315);
                reload();
                sleep(2000);
                telemetry.addLine("case 1 finished");
                telemetry.update();
                autoStep++;

            case 2:
                Shoot(315);
                reload();
                sleep(2000);
                stopMotors();
                telemetry.addLine("case 2 is finished");
                telemetry.update();


                break;


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
            //packet.put("Imu heading:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            dashboard.sendTelemetryPacket(packet);
        telemetry.addData("ShootMotor rpm:", ShootMotor.getVelocity());
        telemetry.addData("PID Coefficients:", ShootMotor.getPIDFCoefficients(ShootMotor.getMode()));
        telemetry.addData("Servo1 position:", servo1.getPosition());
        telemetry.addData("Servo2 position:", servo2.getPosition());
        telemetry.addData("Left motor power:", leftDrive.getPower());
        telemetry.addData("Right motor power:", rightDrive.getPower());
        //telemetry.addData("Imu heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

}