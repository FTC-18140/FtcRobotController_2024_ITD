package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Intake {
    Telemetry telemetry;
    Servo wristLeft = null;
    Servo wristRight = null;
    CRServo spinner = null;
    DcMotor arm = null;
    DcMotor elbow = null;
    private PIDController controller;
    public static double p = 0.007, i = 0, d = 0;
    public static double f = 0.2;

    public final double WRIST_INIT = 0.0;
    public final double WRIST_MIN = 0.0;
    public final double WRIST_MAX = 0.1;
    public final double ELBOW_MIN = 0.0;
    public final double ELBOW_MAX = 5.0;
    public final double ARM_MIN = 0;
    public final double ARM_MAX = 50;
    public final double WRIST_RIGHT_MIN = -5.0;
    public final double WRIST_RIGHT_MAX = 5.0;

    public static double ticks_in_degree = 5.96;
    public static double target = 0;
    public double armPos;
    public double elbowPosition;
    public double wristLeftPos;
    public double wristRightPos;
    public double spinnerPos;

    // Lift parameters
    final private double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    final private double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (SPOOL_DIAMETER_CM * Math.PI);

    public enum Positions{
        READY_TO_INTAKE(0.0,0.0,0.0);
        public final double wristLeftPos;
        public final double wristRightPos;
        public final double spinnerPos;
        Positions(double wristl, double wristr, double spin){
            wristLeftPos = wristl;
            wristRightPos = wristr;
            spinnerPos = spin;
        }
    }
    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        controller = new PIDController(p, i, d);

        //initialize servos and motors
        try{
            wristLeft = hwMap.servo.get("wristLeft");
            wristLeft.setDirection(Servo.Direction.REVERSE);
            wristLeftPos = wristLeft.getPosition();
        }catch(Exception e){
            telemetry.addData("wristLeft servo not found in configuration",0);
        }
        try{
            wristRight = hwMap.servo.get("wristRight");
            wristRightPos = wristRight.getPosition();
        }catch(Exception e){
            telemetry.addData("wristRight servo not found in configuration",0);
        }
        try{
            spinner = hwMap.get(CRServo.class, "spinner");

        }catch(Exception e){
            telemetry.addData("spinner servo not found in configuration",0);
        }
        try{
            elbow = hwMap.dcMotor.get("elbow");

            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }catch (Exception e){
            telemetry.addData("elbow motor not found in configuration",0);
        }
        try{
            arm = hwMap.dcMotor.get("arm");

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }catch(Exception e){
            telemetry.addData("arm motor not found in configuration", 0);
        }

    }
    public void armUp(double power){
        telemetry.addData("arm position : ", armPos/COUNTS_PER_CM);
        if(armPos/COUNTS_PER_CM <=28){
            arm.setPower(power);
        }
        else{
            armStop();
        }
    }
    public void armDown(double power){
        telemetry.addData("arm position : ", armPos/COUNTS_PER_CM);
        if(armPos/COUNTS_PER_CM >=0){
            arm.setPower(power);
        }
        else{
            armStop();
        }
    }
    public void armStop(){
        arm.setPower(0);
    }
    public void elbowUp(double power) {
        telemetry.addData("elbow position : ", elbowPosition/COUNTS_PER_CM);
        if(target < 640){
            target+=5;
        }

    }
    public void elbowDown(double power) {
        telemetry.addData("elbow position : ", elbowPosition/COUNTS_PER_CM);
        if(target >= 30){
            target-=5;
        }

    }
    public void elbowStop(){
        //elbow.setPower(0);
    }
    public void wristMove(double position) {
        wristLeft.setPosition(position);
        wristRight.setPosition(position);
    }
    public void spin(double power){
        spinner.setPower(power);
    }
    public void spinStop(){
        spinner.setPower(0);
    }
    public void  update(){
        controller.setPID(p, i, d);
        elbowPosition = elbow.getCurrentPosition();
        double pid = controller.calculate(elbowPosition, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double power = pid + ff;
        if(target<30){
            elbow.setPower(0);
        }else {
            elbow.setPower(power);
        }
        telemetry.addData("power : ", power);

        wristLeftPos = wristLeft.getPosition();
        wristRightPos = wristRight.getPosition();
        armPos = arm.getCurrentPosition();

        telemetry.addData("elbowPos : ", elbowPosition);
        telemetry.addData("targetPos : ", target);
        telemetry.update();
    }
}
