package org.firstinspires.ftc.teamcode.Robot;


import static com.qualcomm.robotcore.util.Range.clip;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Intake {
    Telemetry telemetry;
    Servo wrist = null;
    Servo claw = null;
    CRServo spinner = null;
    DcMotor arm = null;
    DcMotor elbow = null;

    public double clawPos;

    public double armTo = 0;
    double armTarget = 0;

    ColorSensor colorL = null;
    ColorSensor colorR = null;
    float hsvValues[] = {0,0,0};
    float hsvValuesL[] = {0,0,0};
    float hsvValuesR[] = {0,0,0};
    private PIDController controller;
    public static double p = 0.05, i = 0, d = 0.00025;

    public static double factor_p_down = 0.09;
    public static double factor_d_down = 1.25;
    public static double f = 0.025;
    public static double fSin = 0.025;
    public static double pDown = 0.01, iDown = 0, dDown = 0.0001;


    public static double fDown = 0.25;

    public final double WRIST_INIT = 0.0;
    public final double WRIST_MIN = 0.0;
    public final double WRIST_MAX = 1.0;
    public static double CLAW_MIN = 0;
    public static double CLAW_MAX = 0.35;
    public final double ELBOW_MIN = 0;
    public final double ELBOW_MIN_SLOW = 30;

    public static double ELBOW_MAX = 105;
    public static double ELBOW_LOW = 55;
    public static double ELBOW_HIGH_CHAMBER = 50;
    public static double ELBOW_HIGH_CHAMBER_SCORING = 41;

    public int elbowDirection = 0;
    public final double ARM_MIN = 0;
    public static double ARM_MAX = 42;
    public static double ARM_MAX_HORIZONTAL = 32;

    public double arm_offset = 0;
    public boolean arm_override = false;

    public static double ticks_in_degree = 21.64166666666667;

    public final double ELBOW_GEAR_RATIO = 5.23 * 5.23 * 5.23; // We are using three 5:1 slices
    public final double ELBOW_SPROCKET_RATIO = 28.0/14.0; // We are using a 14-tooth drive sprocket and a 28-tooth driven sprocket
    public final double ELBOW_TICKS_PER_MOTOR_REV = 28.0;
    public final double COUNTS_PER_ELBOW_REV = ELBOW_TICKS_PER_MOTOR_REV * ELBOW_GEAR_RATIO * ELBOW_SPROCKET_RATIO;
    public final double COUNTS_PER_ELBOW_DEGREE = ticks_in_degree;


//    public final double ELBOW_SPROCKET_RATIO = 28.0/14.0; // We are using a 14-tooth drive sprocket and a 28-tooth driven sprocket
//    public final double COUNTS_PER_ELBOW_MOTOR_REV = 3895.9;  // This is the PPR for a 43 RPM goBilda motor
//    public final double COUNTS_PER_ELBOW_REV = COUNTS_PER_ELBOW_MOTOR_REV  * ELBOW_SPROCKET_RATIO;
//    public final double COUNTS_PER_ELBOW_DEGREE = COUNTS_PER_ELBOW_REV / 360.0;

    public static double target = 0;
    public static double directSetTarget = 0;
    public double armPos;
    public double elbowPosition;
    public double wristPos;
    public double spinnerPos;

    // Lift parameters
    final private double COUNTS_PER_ARM_MOTOR_REV = 28; // REV HD Hex motor
    final private double ARM_MOTOR_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double ARM_SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_ARM_CM = (COUNTS_PER_ARM_MOTOR_REV * ARM_MOTOR_GEAR_REDUCTION)
            / (ARM_SPOOL_DIAMETER_CM * Math.PI);


    public enum Positions{
        READY_TO_INTAKE(0.5,1.0,0, CLAW_MAX),
        LOW_BASKET(0.7,ARM_MAX_HORIZONTAL,ELBOW_LOW, CLAW_MAX),
        HIGH_CHAMBER(0.3,20, ELBOW_HIGH_CHAMBER, CLAW_MAX),
        HIGH_CHAMBER_SCORING(0,25, ELBOW_HIGH_CHAMBER_SCORING, CLAW_MAX),
        HIGH_CHAMBER_SCORING_AUTO(0.1,25, ELBOW_HIGH_CHAMBER_SCORING, CLAW_MAX),
        INTAKE_SPECIMEN(0.27, 5, 13, CLAW_MIN),
        //Max elbow, Max arm extend, base of intake parallel with floor ↓
        HIGH_BASKET(0.26,ARM_MAX,ELBOW_MAX, CLAW_MAX);
        public final double wristPos;
        public final double armPos;
        public final double elbowPos;
        public final double clawPos;
        Positions(double wrist, double arm, double elbow, double claw){
            wristPos = wrist;
            armPos = arm;
            elbowPos = elbow;
            clawPos = claw;
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
            claw = hwMap.servo.get("claw");
            claw.setPosition(CLAW_MAX);
        }catch(Exception e){
            telemetry.addData("claw servo not found in configuration",0);
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
            elbow.setDirection(DcMotorSimple.Direction.REVERSE);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }catch (Exception e){
            telemetry.addData("elbow motor not found in configuration",0);
        }
        try{
            arm = hwMap.dcMotor.get("arm");

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            arm.setPower(0.8);

        }catch(Exception e){
            telemetry.addData("arm motor not found in configuration", 0);
        }

    }
    public void init_loop(){
        update();
    }
    public void start(){
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void preset(Positions position){
        armPos = arm.getCurrentPosition() / COUNTS_PER_ARM_CM;
        setElbowTo(position.elbowPos);
        wristMove(position.wristPos);
        clawMove(position.clawPos);
        armTarget = position.armPos;
        armTo = armTarget - (armPos - arm_offset);
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
            //Creates hsvValues equal to the highest of the colors detected by the left and right sensors
            if(hsvValuesL[2] >= hsvValuesR[2]){
                hsvValues = hsvValuesL;
            }else{
                hsvValues = hsvValuesR;
            }
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
    public void clawMove(double position){
        if(position<=CLAW_MAX || position>=CLAW_MIN) {
            claw.setPosition(position);
        }
    }
    public void armUp(double power) {
        telemetry.addData("arm position : ", armPos - arm_offset);
        if (target < 30) {
            if (armPos - arm_offset <= ARM_MAX_HORIZONTAL) {
                telemetry.addData("arm position : ", armPos - arm_offset);
                arm.setPower(power);
            } else {
                armStop();
            }
        } else {
            if (armPos - arm_offset <= ARM_MAX) {
                arm.setPower(power);
            } else {
                armStop();
            }
        }
    }
            public void armDown ( double power){
                telemetry.addData("arm position : ", armPos / COUNTS_PER_ARM_CM);
                if(!arm_override) {
                    if (armPos - arm_offset >= ARM_MIN) {
                        arm.setPower(power);
                    } else {
                        armStop();
                    }
                }else{
                    arm.setPower(power);
                }
            }
            public void armStop () {
                arm.setPower(0);
            }
            public void elbowUp ( double power){
                telemetry.addData("elbow position : ", elbowPosition / COUNTS_PER_ARM_CM);
                if (target + power <= ELBOW_MAX) {
                    target += power;
                } else {
                    target = ELBOW_MAX;
                }
                directSetTarget = target;
                elbowDirection = 0;
            }
            public void elbowDown ( double power){
                telemetry.addData("elbow position : ", elbowPosition / COUNTS_PER_ARM_CM);
                if (target - power >= ELBOW_MIN) {
                    target -= power;
                } else {
                    target = ELBOW_MIN;
                }
                directSetTarget = target;
                elbowDirection = 0;
            }
            public void setElbowTo ( double position){
                if (position <= ELBOW_MAX && position >= ELBOW_MIN) {
                    if (position < target) {
                        elbowDirection = -1;
                    } else if (position > target) {
                        elbowDirection = 1;
                    } else {
                        elbowDirection = 0;
                    }
                    directSetTarget = position;
                }
            }
            public void elbowStop () {
                //elbow.setPower(0);
            }
            public void wristMove ( double position){
                wrist.setPosition(position);
            }
            public void spin ( double power){
                spinner.setPower(power);
            }
            public void spinStop () {
                spinner.setPower(0);
            }

            public Action armUpAction ( double position){
                return new Action() {
                    private double pos = position;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        double currentPos = arm.getCurrentPosition();
                        armTarget = pos;
                        armTo = 0;
                        if (pos >= currentPos / COUNTS_PER_ARM_CM) {
                            armUp(1);
                        } else {
                            armStop();
                            return false;
                        }

                        telemetry.addData("currentPos(armAction): ", currentPos);
                        return true;
                    }
                };
            }
            public Action armDownAction ( double position){
                return new Action() {
                    private double pos = position;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        double currentPos = arm.getCurrentPosition();
                        armTarget = pos;
                        armTo = 0;
                        if (pos <= currentPos / COUNTS_PER_ARM_CM) {
                            armDown(-1);
                        } else {
                            armStop();
                            return false;
                        }

                        telemetry.addData("currentPos(armAction): ", currentPos);
                        return true;
                    }
                };
            }

            public Action wristMoveAction ( double position){
                return new Action() {
                    private double pos = position;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        wristMove(pos);
                        return Math.abs(wristPos - pos) > 0.1;
                    }
                };
            }
            public Action spinnerAction ( double power){
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
            public Action elbowAction ( double position){
                return new Action() {
                    private double pos = position;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        setElbowTo(pos);
                        return false;
                    }
                };
            }
            public Action clawAction (double position){
                return new Action() {
                    private double pos = position;
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        clawMove(pos);
                        return false;
                    }
                };
            }
            public Action scoreSpecimenFromStart(Pose2d pose, ThunderBot2024 robot){
                return new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        return false;
                    }
                };
            }
            public Action checkForSample (String color,double limit){
                return new Action() {
                    String c = color;
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (c.contains(getCalculatedColor()) || timer.seconds() >= limit) {
                            return false;
                        }
                        return true;
                    }
                };
            }
            public Action updateAction () {
                return new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        update();
                        return true;
                    }
                };
            }
            //
            public void update ()
            {
                /////////////////////////
                // Update telescoping arm
                /////////////////////////
                armPos = arm.getCurrentPosition() / COUNTS_PER_ARM_CM;
                if (armTo > 0) {
                    if (armPos - arm_offset < armTarget) {
                        armUp(0.3);
                    } else {
                        armTo = 0;
                    }
                } else if (armTo < 0) {
                    if (armPos - arm_offset > armTarget) {
                        armDown(-1.0);
                    } else {
                        armTo = 0;
                    }
                }

                //        telemetry.addData("arm direction for preset ", armTarget-armPos/COUNTS_PER_CM);
//        telemetry.addData("arm target: ", armTarget);
//        telemetry.addData("armPos: ", armPos);

                ////////////////
                // Update elbow
                ////////////////
                target = directSetTarget;

                if (target < ELBOW_MIN) {
                    target = ELBOW_MIN;
                }
                elbowPosition = elbow.getCurrentPosition();

                //controller.setPID(p,i,d);
                double ff = f * Math.cos(Math.toRadians(clip(elbowPosition / COUNTS_PER_ELBOW_DEGREE, 0, 180)));
                if (target >= elbowPosition/COUNTS_PER_ELBOW_DEGREE) {
                    // the cosine lowers as it approaches half-PI/90°
                    // the sine balances out the cosine, allowing the arm to raise fully
                    ff = f * Math.cos(Math.toRadians(clip(elbowPosition / COUNTS_PER_ELBOW_DEGREE, 0, 180))) + fSin * Math.sin(Math.toRadians(clip(elbowPosition / COUNTS_PER_ELBOW_DEGREE, 0, 180)));
                    controller.setPID(p, i, d);
                } else {
                    double pDown = Math.abs(p * factor_p_down * Math.cos(Math.toRadians(clip(elbowPosition / COUNTS_PER_ELBOW_DEGREE, 0, 180))));
                    double dDown = Math.abs(d * factor_d_down * Math.cos(Math.toRadians(clip(elbowPosition / COUNTS_PER_ELBOW_DEGREE, 0, 180))));
                    controller.setPID(pDown, i, dDown);
                }
                double pid = controller.calculate(elbowPosition / COUNTS_PER_ELBOW_DEGREE, target);

                double power = pid + ff;
                elbow.setPower(power);

                clawPos = claw.getPosition();
//                telemetry.addData("clawPos: ", claw.getPosition());
                telemetry.addData("power : ", power);
                telemetry.addData("ff : ", ff);
                telemetry.addData("pid : ", pid);
                telemetry.addData("target : ", target);
                telemetry.addData("elbowpos : ", elbowPosition);
                telemetry.addData("elbowpos in degrees: ", elbowPosition / COUNTS_PER_ELBOW_DEGREE);

                wristPos = wrist.getPosition();

                // Look at sample color
                calculateSensorValues();


//        if(hsvValues[2] < 2000){
//            telemetry.addData("color detected: ", "none");
//        }else if(hsvValues[0] < 70){
//            telemetry.addData("color detected: ", "red");
//        } else if (hsvValues[0] < 150) {
//            telemetry.addData("color detected: ", "yellow");
//        } else if (hsvValues[0] < 200) {
//            telemetry.addData("color detected: ", "none");
//        }else{
//            telemetry.addData("color detected: ", "blue");
//        }
//        telemetry.addData("incrementing target: ", directSetTarget);
//
//        telemetry.addData("hue", hsvValues[0]);
//        telemetry.addData("value", hsvValues[2]);

                //telemetry.update();
            }
}
