package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor liftLeft;
    DcMotor liftRight;
    Servo leftServo;
    Servo rightServo;

    public double offsetPos;

    public final double LIFT_MAX = 790;
    public final double LIFT_SERVO_MAX = 0.1;
    public final double LIFT_SERVO_MAX_R = 0.3;
    public final double LIFT_SERVO_LIFT = 0.1;

    public double lift_target = 0;
    public void init(HardwareMap hwMap, Telemetry telem, double startPos){
        offsetPos = startPos;
        hardwareMap = hwMap;
        telemetry = telem;
        try{
            liftLeft = hardwareMap.dcMotor.get("liftL");
            liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }catch (Exception e){
            telemetry.addData("left lift motor not found in configuration",0);
        }
        try{
            liftRight = hardwareMap.dcMotor.get("liftR");
            //liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }catch (Exception e){
            telemetry.addData("right lift motor not found in configuration",0);
        }
        try{
            leftServo = hardwareMap.servo.get("liftServoL");
            leftServo.setDirection(Servo.Direction.REVERSE);
            leftServo.setPosition(LIFT_SERVO_MAX);
        }catch (Exception e){
            telemetry.addData("'liftServoL' not found in configuration", 0);
        }
        try{
            rightServo = hardwareMap.servo.get("liftServoR");
            rightServo.setDirection(Servo.Direction.REVERSE);
            rightServo.setPosition(LIFT_SERVO_MAX_R);
        }catch (Exception e){
            telemetry.addData("'liftServoR' not found in configuration", 0);
        }
    }
    public double getLiftPosR(){
        return liftRight.getCurrentPosition()+offsetPos;
    }
    public double getLiftPosL(){
        return liftLeft.getCurrentPosition()+offsetPos;
    }
    public double getLeftServoPos(){
        return leftServo.getPosition();
    }
    public double getRightServoPos(){
        return rightServo.getPosition();
    }
    public void moveLift(double power){
        if(power > 0){
            if(getLiftPosL() >= LIFT_MAX){
                liftLeft.setPower(0);
            }else{
                liftLeft.setPower(power);
            }
            if(getLiftPosR() >= LIFT_MAX){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        }else if(power < 0){
            if(getLiftPosL() <= 1){
                liftLeft.setPower(0);
            }else{
                liftLeft.setPower(power);
            }
            if(getLiftPosR() <= 1){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        } else{
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }

    }
    public void moveToTop(){
        leftServo.setPosition(-0.3);
        rightServo.setPosition(-0.3);
        lift_target = LIFT_MAX;
    }
    public void moveToMin(){
        leftServo.setPosition(LIFT_SERVO_MAX);
        rightServo.setPosition(LIFT_SERVO_MAX_R);
        lift_target = 0;
    }
    public boolean liftUpTo(double position){
        if(Math.abs(position-getLiftPosR())<=1){
            moveLift(0);
            telemetry.addData("at the top", 0);
            return false;
        }else{
            moveLift(1);
            telemetry.addData("going up",0);
            return true;
        }

    }
    public boolean liftDownTo(double position){
        if(Math.abs(getLiftPosR()-position)<=1){
            moveLift(0);
            telemetry.addData("at the bottom", 0);
            return false;
        }else{
            moveLift(-1);
            telemetry.addData("going down", 0);
            return true;
        }

    }
    public Action liftToAction(double position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return liftUpTo(position);
            }
        };
    }
    public void update(){
        if(lift_target < 50){
            liftDownTo(lift_target);
            telemetry.addData("lift up", 0);
        }else if(lift_target > 500){
            liftUpTo(lift_target);
            telemetry.addData("lift down", 0);
        }

    }
}
