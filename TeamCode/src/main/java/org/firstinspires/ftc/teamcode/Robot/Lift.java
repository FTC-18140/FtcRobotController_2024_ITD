package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor liftLeft;
    DcMotor liftRight;

    public final double LIFT_MAX = 100;
    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;
        try{
            liftRight = hardwareMap.dcMotor.get("liftR");
        }catch (Exception e){
            telemetry.addData("right lift motor not found in configuration",0);
        }
    }
    public void moveLift(double power){
        liftRight.setPower(power);
    }
}
