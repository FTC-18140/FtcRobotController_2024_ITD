package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    Telemetry telemetry;
    Servo led = null;
    ElapsedTime ledTimer = new ElapsedTime();
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
        if(ledTimer.seconds() < 90){
            if(led != null){
                led.setPosition(-0.99);
            }
        }
    }
}
