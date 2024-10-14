package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    Servo wristLeft = null;
    Servo wristRight = null;
    Servo spinner = null;
    DcMotorSimple arm = null;

    public double wristLeftPos;
    public double wristRightPos;
    public double spinnerPos;

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

    }
}
