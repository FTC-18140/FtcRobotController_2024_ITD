package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    Telemetry telemetry;
    Servo led = null;
    public ElapsedTime ledTimer = new ElapsedTime();
    public double red = 0.28;
    public double blue = 0.62;
    public double green = 0.5;
    public double yellow = 0.388;
    public double purple = 0.72;
    public double white = 1;
    public void init(HardwareMap hwMap, Telemetry telem){
        telemetry = telem;
        try{
            led = hwMap.servo.get("led");
            led.setPosition(red);
        }catch (Exception e){
            telemetry.addData("led not found in configuration", 0);
        }
    }
    public void setToColor(String color) {
        if(led != null) {
            switch(color){
                case("red"):
                    led.setPosition(red);
                    break;
                case("yellow"):
                    led.setPosition(yellow);
                    break;
                case("blue"):
                    led.setPosition(blue);
                    break;
                default:
                    led.setPosition(white);
                    break;
            }

        }
    }
}
