package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    Telemetry telemetry;
    Servo led = null;
    public ElapsedTime ledTimer = new ElapsedTime();
    public void init(HardwareMap hwMap, Telemetry telem){
        telemetry = telem;
        try{
            led = hwMap.servo.get("led");
            led.setPosition(-0.99);
        }catch (Exception e){
            telemetry.addData("led not found in configuration", 0);
        }
    }
    public void update() {
        if(led != null) {
            if (ledTimer.seconds() < 90) {
                led.setPosition(-0.99);
            } else if (ledTimer.seconds() < 105) {
                led.setPosition(0.67);
            } else {
                led.setPosition(0.61);
            }
        }
    }
}
