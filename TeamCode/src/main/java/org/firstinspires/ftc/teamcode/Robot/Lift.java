package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotorEx liftLeft;
    DcMotorEx liftRight;
    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;
    }
}
