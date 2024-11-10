package org.firstinspires.ftc.teamcode.Robot;


import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Intake {
    Telemetry telemetry;
    Servo wrist = null;
    CRServo spinner = null;
    DcMotor arm = null;
    DcMotor elbow = null;

    public double armTo = 0;
    double armTarget = 0;

    ColorSensor colorL = null;
    ColorSensor colorR = null;
    float hsvValues[] = {0,0,0};
    float hsvValuesL[] = {0,0,0};
    float hsvValuesR[] = {0,0,0};
    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0.0001;
    public static double pDown = 0.01, iDown = 0, dDown = 0.0001;

    public static double f = 0.015;
    public static double fDown = 0.25;

    public final double WRIST_INIT = 0.0;
    public final double WRIST_MIN = 0.0;
    public final double WRIST_MAX = 1.0;
    public final double ELBOW_MIN = 0;
    public final double ELBOW_MIN_SLOW = 700;
    public final double ELBOW_MAX = 2400;
    public int elbowDirection = 0;
    public final double ARM_MIN = 0;
    public final double ARM_MAX = 28;
    public final double WRIST_RIGHT_MIN = -5.0;
    public final double WRIST_RIGHT_MAX = 5.0;

    public static double ticks_in_degree = 5.96;
    public static double target = 0;
    public double directSetTarget = 0;
    public double armPos;
    public double elbowPosition;
    public double wristLeftPos;
    public double wristPos;
    public double spinnerPos;

    // Lift parameters
    final private double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    final private double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (SPOOL_DIAMETER_CM * Math.PI);


    public enum Positions{
        READY_TO_INTAKE(0.5,0.0,0.0),
        //Max elbow, Max arm extend, base of intake parallel with floor ↓
        HIGH_BASKET(0.3,28,2400);
        public final double wristPos;
        public final double armPos;
        public final double elbowPos;
        Positions(double wrist, double arm, double elbow){
            wristPos = wrist;
            armPos = arm;
            elbowPos = elbow;
        }
    }
    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        controller = new PIDController(p, i, d);

        //Initialize Sensors
        try{
            colorL = hwMap.colorSensor.get("colorL");
        }catch(Exception e){
            telemetry.addData("left color sensor not found in configuration",0);
        }
        try{
            colorR = hwMap.colorSensor.get("colorR");
        }catch(Exception e){
            telemetry.addData("right color sensor not found in configuration",0);
        }
        //initialize servos and motors
        try{
            wrist = hwMap.servo.get("wrist");
            wrist.setDirection(Servo.Direction.REVERSE);
            wristPos = wrist.getPosition();
            wrist.setPosition(WRIST_INIT);
        }catch(Exception e){
            telemetry.addData("wristRight servo not found in configuration",0);
        }
        try{
            spinner = hwMap.get(CRServo.class, "spinner");

        }catch(Exception e){
            telemetry.addData("spinner servo not found in configuration",0);
        }
        try{
            elbow = hwMap.dcMotor.get("elbow");

            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }catch (Exception e){
            telemetry.addData("elbow motor not found in configuration",0);
        }
        try{
            arm = hwMap.dcMotor.get("arm");

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }catch(Exception e){
            telemetry.addData("arm motor not found in configuration", 0);
        }

    }

    public void preset(Positions position){
        armPos = arm.getCurrentPosition();
        setElbowTo(position.elbowPos);
        wristMove(position.wristPos);
        armTarget = position.armPos;
        armTo = armTarget - armPos/COUNTS_PER_CM;
        telemetry.addData("preset arm: ", armTo);
    }
    public Action presetAction(Positions position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                preset(position);
                return false;
            }
        };
    }
    public void calculateSensorValues(){
        if(colorL != null) {
            Color.RGBToHSV((int) (colorL.red() * 255), (int) (colorL.green() * 255), (int) (colorL.blue() * 255), hsvValuesL);
        }
        if(colorR != null) {
            Color.RGBToHSV((int) (colorR.red() * 255), (int) (colorR.green() * 255), (int) (colorR.blue() * 255), hsvValuesR);
        }
        if(colorL != null && colorR != null){
            //Creates hsvValues equal to the average of the colors detected by the left and right sensors
            hsvValues[0] = Math.round((hsvValuesL[0] + hsvValuesR[0])/2);//hue
            hsvValues[1] = Math.round((hsvValuesL[1] + hsvValuesR[1])/2);//saturation
            hsvValues[2] = Math.round((hsvValuesL[2] + hsvValuesR[2])/2);//light value

        }
    }
    public String getCalculatedColor(){
        calculateSensorValues();
        if(hsvValues[2] < 1000){
            //light values of less than 1000 indicate black, the inside of the intake
            return "none";
        }else if(hsvValues[0] < 75){
            return "red";
        } else if (hsvValues[0] < 150) {
            return "yellow";
        } else if (hsvValues[0] < 200) {
            return "none";
        }else{
            return "blue";
        }
    }
    public void armUp(double power){
        telemetry.addData("arm position : ", armPos/COUNTS_PER_CM);
        if(armPos/COUNTS_PER_CM <=ARM_MAX){
            arm.setPower(power);
        }
        else{
            armStop();
        }
    }
    public void armDown(double power){
        telemetry.addData("arm position : ", armPos/COUNTS_PER_CM);
        if(armPos/COUNTS_PER_CM >=ARM_MIN){
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
        if(target+power <= ELBOW_MAX){
            target+=power;
        }else{
            target = ELBOW_MAX;
            directSetTarget = target;
            elbowDirection = 0;
        }

    }
    public void elbowDown(double power) {
        telemetry.addData("elbow position : ", elbowPosition/COUNTS_PER_CM);
        if(target-power >= ELBOW_MIN){
            target-=power;
        }else{
            target = ELBOW_MIN;
        }
        directSetTarget = target;
        elbowDirection = 0;
    }
    public void setElbowTo(double position){
        if(position <= ELBOW_MAX && position >= ELBOW_MIN){
            if(position < target){
                elbowDirection = -1;
            } else if (position > target) {
                elbowDirection = 1;
            } else {
                elbowDirection = 0;
            }
            directSetTarget = position;
        }
    }
    public void elbowStop(){
        //elbow.setPower(0);
    }
    public void wristMove(double position) {
        wrist.setPosition(position);
    }
    public void spin(double power){
        spinner.setPower(power);
    }
    public void spinStop(){
        spinner.setPower(0);
    }

    public Action armUpAction(double position){
        return new Action() {
            private double pos = position;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double currentPos = arm.getCurrentPosition();
                if (pos >= currentPos / COUNTS_PER_CM) {
                    armUp(1);
                } else {
                    armStop();
                    return false;
                }

                telemetry.addData("currentPos(armAction): ",currentPos);
                return true;
            }
        };
    }
    public Action armDownAction(double position){
        return new Action() {
            private double pos = position;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double currentPos = arm.getCurrentPosition();
                if (pos <= currentPos / COUNTS_PER_CM) {
                    armDown(-1);
                } else {
                    armStop();
                    return false;
                }

                telemetry.addData("currentPos(armAction): ",currentPos);
                return true;
            }
        };
    }

    public Action wristMoveAction(double position){
        return new Action() {
            private double pos = position;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wristMove(pos);
                return Math.abs(wristPos -pos)>0.1;
            }
        };
    }
    public Action spinnerAction(double power){
        return new Action() {
            private double pow = power;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                spin(pow);
                telemetry.addData("spinnerAction: ", 0);
                return false;
            }
        };
    }
    public Action elbowAction(double position){
        return new Action() {
            private double pos = position;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setElbowTo(pos);
                return false;
            }
        };
    }
    public Action checkForSample(String color, double limit){
        return new Action() {
            String c = color;
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(c.contains(getCalculatedColor()) || timer.time()>=limit){
                    return false;
                }
                return true;
            }
        };
    }
    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return true;
            }
        };
    }
    public void  update(){
        armPos = arm.getCurrentPosition();
        if(armTo > 0){
            if(armPos / COUNTS_PER_CM < armTarget){
                armUp(0.4);
            }else{
                armTo = 0;
            }
        } else if (armTo < 0) {
            if(armPos / COUNTS_PER_CM > armTarget){
                armDown(-1.0);
            }else {
                armTo = 0;
            }
        }

        if(elbowDirection == 1){
            target = directSetTarget;
        } else if (elbowDirection == -1) {
            if(directSetTarget < ELBOW_MIN_SLOW){
                if(target > ELBOW_MIN_SLOW){
                    telemetry.addData("go to slowdown",0);
                    target = ELBOW_MIN_SLOW;
                }else if(target > directSetTarget){
                    telemetry.addData("slow down", 0);
                    target -= 20;
                }
            }else{
                target = directSetTarget;
            }
        }
        if(target < ELBOW_MIN){
            target = ELBOW_MIN;
        }
        calculateSensorValues();
        elbowPosition = elbow.getCurrentPosition();

        controller.setPID(p,i,d);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
//        if(target >=elbowPosition){
//            controller.setPID(p, i, d);
//            ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
//        }else{
//            controller.setPID(pDown,iDown,dDown);
//            ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * fDown;
//        }
        double pid = controller.calculate(elbowPosition, target);

        double power = pid + ff;
        power = Range.clip(power, -0.5, 0.7);
        if(target<ELBOW_MIN){
            elbow.setPower(0);
        }else {
            elbow.setPower(power);
        }
        telemetry.addData("power : ", power);

        wristPos = wrist.getPosition();


        if(hsvValues[2] < 2000){
            telemetry.addData("color detected: ", "none");
        }else if(hsvValues[0] < 70){
            telemetry.addData("color detected: ", "red");
        } else if (hsvValues[0] < 150) {
            telemetry.addData("color detected: ", "yellow");
        } else if (hsvValues[0] < 200) {
            telemetry.addData("color detected: ", "none");
        }else{
            telemetry.addData("color detected: ", "blue");
        }
        telemetry.addData("incrementing target: ", directSetTarget);

        telemetry.addData("arm direction for preset ", armTarget-armPos/COUNTS_PER_CM);
        telemetry.addData("arm target: ", armTarget);
        telemetry.addData("hue", hsvValues[0]);
        telemetry.addData("value", hsvValues[2]);
        telemetry.addData("elbowPos : ", elbowPosition);
        telemetry.addData("targetPos : ", target);
        telemetry.update();
    }
}
