package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;
@TeleOp
@Config
public class Teleop_bot extends OpMode {
    public static double kP = 34;
    public static double kI = 0.007;
    public static double kD = 1.4;
    public static double kF = 14;
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotorEx ShootMotor;
    Servo servo1;
    Servo servo2;
    IMU imu;
    WebcamName webcam;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;


    @Override
    public void init() {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Create AprilTag processor
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        // Create VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .build();

        telemetry.addLine("Initializing camera...");

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
        //telemetry.addData("servo1 position", servo1.getPosition());
        //telemetry.addData("servo2 position", servo2.getPosition());
        ShootMotor = hardwareMap.get(DcMotorEx.class, "ShootMotor");
        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ShootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
        IMU.Parameters parameters = new IMU.Parameters
                (new RevHubOrientationOnRobot
                        (RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        telemetry.addData("Status", "Initalized");
    }


    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Timer to track match duration
    private ElapsedTime runtime = new ElapsedTime();
    private boolean rumbleTriggered = false;
    private final long matchDurationMs = 2 * 60 * 1000;  // 2 minutes
    private final long rumbleBeforeEndMs = 30 * 1000;    // 30 seconds before end


    @Override
    public void loop() {
        // --- DRIVE ---
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        double speedScale = 0.7;
        if (gamepad1.left_bumper) {
            speedScale = 1.0;
        } else if (gamepad1.right_bumper) {
            speedScale = 0.4;
        }


        leftPower *= speedScale;
        rightPower *= speedScale;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);


        List<AprilTagDetection> detections = tagProcessor.getDetections();

        try {


            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(1);

                double x = tag.ftcPose.x;          // left/right offset (inches)
                double y = tag.ftcPose.y;          // forward/back distance (inches)
                double heading = tag.ftcPose.yaw;  // rotation needed (degrees)

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X Offset (in)", x);
                telemetry.addData("Y Distance (in)", y);
                telemetry.addData("Yaw (deg)", heading);

                // --- ALIGNMENT LOGIC ---
                double strafePower = x * 0.5;     // tune this
                double turnPower = heading * 0.5; // tune this

                telemetry.addData("Strafe Power", strafePower);
                telemetry.addData("Turn Power", turnPower);
            } else {
                telemetry.addLine("No AprilTag detected");
            }
        } catch (IndexOutOfBoundsException e) {
            telemetry.addLine("No apriltag seen");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("returned null for tag pose");
            telemetry.update();


            //ATTACHMENTS


            //Shooting
            if (gamepad2.right_trigger > 0.9) {
                ShootMotor.setVelocity(375);

            } else {
                ShootMotor.setVelocity(0);

            }
            //organizer forwards
            if (gamepad2.circle) {
                servo1.setDirection(Servo.Direction.FORWARD);
                servo2.setDirection(Servo.Direction.REVERSE);
                servo1.setPosition(1);
                servo2.setPosition(1);
            } else {
                servo1.setPosition(0.0);
                servo2.setPosition(0.0);
            }
            //organizer backwards
            if (gamepad2.square) {
                servo1.setPosition(0);
                servo2.setPosition(0);
            }
            long elapsedMs = (long) runtime.milliseconds();
            long remainingMs = matchDurationMs - elapsedMs;

            // Trigger rumble only once when 30 seconds are left
            if (!rumbleTriggered && remainingMs <= rumbleBeforeEndMs) {
                rumbleTriggered = true;

                // Vibrate for 500ms multiple times until endgame or until safe period for non-blocking
                // Here we just trigger a short pulse as an example
                gamepad1.rumble(1.0, 1.0, 2);

                telemetry.addLine("Rumble triggered for endgame!");


            }
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("power", leftDrive.getPower());
            packet.put("power", rightDrive.getPower());
            packet.put("ShootMotor rpm", ShootMotor.getVelocity());

            packet.put("P", kP);
            packet.put("I", kI);
            packet.put("D", kD);
            packet.put("F", kF);
            packet.put("Servo1 position", servo1.getPosition());
            packet.put("Servo2 position", servo2.getPosition());
            packet.put("imu heading", getHeading());
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Detections", detections.size());
            telemetry.addData("Velocity:", ShootMotor.getVelocity());
            telemetry.addData("Left wheel speed", leftDrive.getPower());
            telemetry.addData("Right wheel speed:", rightDrive.getPower());
            telemetry.addData("servo1 postion", servo1.getPosition());
            telemetry.addData("servo2 postion", servo2.getPosition());
            telemetry.addData("imu heading", getHeading());
            telemetry.update();
        }
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



