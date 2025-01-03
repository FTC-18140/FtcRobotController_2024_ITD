package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public final double LIFT_MAX = 2600;
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
            liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }catch (Exception e){
            telemetry.addData("right lift motor not found in configuration",0);
        }
        try{
            leftServo = hardwareMap.servo.get("liftServoL");
        }catch (Exception e){
            telemetry.addData("'liftServoL' not found in configuration", 0);
        }
        try{
            rightServo = hardwareMap.servo.get("liftServoR");
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
    public void moveLift(double power){
        if(power > 0){
            if(liftLeft.getCurrentPosition()+offsetPos >= LIFT_MAX){
                liftLeft.setPower(0);
            }else{
                liftLeft.setPower(power);
            }
            if(liftRight.getCurrentPosition()+offsetPos >= LIFT_MAX){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        }else if(power < 0){
            if(liftLeft.getCurrentPosition()+offsetPos <= 100){
                liftLeft.setPower(0);
            }else{
                liftLeft.setPower(power);
            }
            if(liftRight.getCurrentPosition()+offsetPos <= 100){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        } else{
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }

    }
    public Action liftTo(double position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveLift(1);
                if(Math.abs(position-getLiftPosR())<=250){
                    moveLift(0);
                    return false;
                }
                return true;
            }
        };
    }
}
