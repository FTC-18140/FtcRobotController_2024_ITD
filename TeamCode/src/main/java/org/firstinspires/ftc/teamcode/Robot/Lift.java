package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor liftLeft;
    DcMotor liftRight;

    public final double LIFT_MAX = 55500;
    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;
        try{
            liftRight = hardwareMap.dcMotor.get("liftR");
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }catch (Exception e){
            telemetry.addData("right lift motor not found in configuration",0);
        }
    }
    public double getLiftPosR(){
        return liftRight.getCurrentPosition();
    }
    public void moveLift(double power){
        if(power > 0){
            if(liftRight.getCurrentPosition() >= LIFT_MAX){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        }else if(power < 0){
            if(liftRight.getCurrentPosition() <= 100){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        } else{
            liftRight.setPower(0);
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
