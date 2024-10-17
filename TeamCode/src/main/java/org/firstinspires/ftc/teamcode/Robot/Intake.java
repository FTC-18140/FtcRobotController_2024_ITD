package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    Servo wristLeft = null;
    Servo wristRight = null;
    Servo spinner = null;
    DcMotor arm = null;
    DcMotor elbow = null;

    public final double WRIST_INIT = 0.0;
    public final double WRIST_MIN = 0.0;
    public final double WRIST_MAX = 0.06;
    public final double ELBOW_MIN = 0.0;
    public final double ELBOW_MAX = 5.0;
    public final double ARM_MIN = 0;
    public final double ARM_MAX = 50;
    public final double WRIST_RIGHT_MIN = -5.0;
    public final double WRIST_RIGHT_MAX = 5.0;

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
            spinner = hwMap.servo.get("spinner");
        }catch(Exception e){
            telemetry.addData("spinner servo not found in configuration",0);
        }
        try{
            elbow = hwMap.dcMotor.get("elbow");

            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if(elbowPosition/COUNTS_PER_CM <=27){
            elbow.setPower(power);
        }
        else{
            elbowStop();
        }
    }
    public void elbowDown(double power) {
        telemetry.addData("elbow position : ", elbowPosition/COUNTS_PER_CM);
        if(elbowPosition/COUNTS_PER_CM >=0){
            elbow.setPower(power);
        }
        else{
            elbowStop();
        }
    }
    public void elbowStop(){
        elbow.setPower(0);
    }
    public void wristMove(double position) {
        wristLeft.setPosition(position);
        wristRight.setPosition(position);
    }
    public void  update(){
        wristLeftPos = wristLeft.getPosition();
        wristRightPos = wristRight.getPosition();
        elbowPosition = elbow.getCurrentPosition();
        armPos = arm.getCurrentPosition();

    }
}
