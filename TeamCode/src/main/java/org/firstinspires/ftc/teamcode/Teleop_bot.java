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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
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
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;


    @Override
    public void init() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .build();

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



    double targetHeading = 0;

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
        // Basic arcade drive
        double drive = gamepad1.left_stick_y;
        double turnInput = gamepad1.right_stick_x;

        double heading = getHeading();  // your IMU function

// ---------------- APRILTAG AUTO-TURN ----------------
        List<AprilTagDetection> detections = tagProcessor.getDetections();


        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            if (tag.ftcPose != null) {
                double tagYaw = tag.ftcPose.yaw;      // rotation error
                double tagDist = tag.ftcPose.z;       // distance to tag (inches)

                // TURN toward the tag
                double turnKp = 0.02;
                double tagTurn = tagYaw * turnKp;

                // DRIVE toward the tag
                double driveKp = 0.05;
                double tagDrive = -tagDist * driveKp;   // negative because z increases as you move away

                // Limit forward speed so it doesn't launch forward
                tagDrive = Math.max(Math.min(tagDrive, 0.4), -0.4);

                // Override driver inputs
                turnInput = tagTurn;
                drive = tagDrive;

                telemetry.addData("AutoDrive", "ACTIVE");
                telemetry.addData("Yaw", tagYaw);
                telemetry.addData("Distance", tagDist);
            } else {
                telemetry.addLine("Tag detected but pose is NULL (too far)");
            }
            if (tag.ftcPose != null) {
                double tagError = tag.ftcPose.yaw;
                double tagTurn = tagError * 0.02;
                turnInput = tagTurn;

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
            } else {
                telemetry.addLine("Tag detected but pose is NULL (too far away)");
            }
        } else {
            telemetry.addLine("No tags detected");
        }

// ----------------------------------------------------


// If the driver is NOT turning AND no AprilTag is controlling turn
        if (Math.abs(turnInput) < 0.1 && Math.abs(drive) > 0.1 && detections.isEmpty()) {
            double error = AngleUnit.normalizeDegrees(targetHeading - heading);
            double kP = 0.005;
            double correction = kP * error;

            turnInput = correction;
        } else if (detections.isEmpty()) {
            // Only update heading target when driver is turning AND no tag is active
            targetHeading = heading;
        }

        double leftPower = drive + turnInput;
        double rightPower = drive - turnInput;

// Normalize
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        double speedScale = 0.7;

        if (gamepad1.left_bumper) {
            speedScale = 1.0;
            telemetry.addLine("Fast mode");
        } else if (gamepad1.right_bumper) {
            speedScale = 0.4;
            telemetry.addLine("Slow mode");
        }

        double leftSpeed = leftPower * speedScale;
        double rightSpeed = rightPower * speedScale;

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);



        //ATTACHMENTS


        //Shooting
        if (gamepad1.right_trigger > 0.9) {
            ShootMotor.setVelocity(320);

        } else {
            ShootMotor.setVelocity(0);

        }
        //organizer forwards
        if (gamepad1.circle) {
            servo1.setDirection(Servo.Direction.FORWARD);
            servo2.setDirection(Servo.Direction.REVERSE);
            servo1.setPosition(1);
            servo2.setPosition(1);
        } else {
            servo1.setPosition(0.0);
            servo2.setPosition(0.0);
        }
        //organizer backwards
        if (gamepad1.square) {
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
        telemetry.addData("Velocity:", ShootMotor.getVelocity());
        telemetry.addData("Left wheel speed", leftDrive.getPower());
        telemetry.addData("Right wheel speed:", rightDrive.getPower());
        telemetry.addData("servo1 postion", servo1.getPosition());
        telemetry.addData("servo2 postion", servo2.getPosition());
        telemetry.addData("imu heading", getHeading());
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



