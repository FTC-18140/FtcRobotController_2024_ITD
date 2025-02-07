package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TBDGamepad.Trigger.LEFT_TRIGGER;
import static org.firstinspires.ftc.teamcode.TBDGamepad.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Auto.AutoPositions;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Teleop")
public class Teleop extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ThunderBot2024 robot = new ThunderBot2024();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    public double wristPos;
    public double clawPos;
    public double spinPos;
    public boolean turning = false;
    public double liftServoPos;

    public double liftPower = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry, 0);
        liftServoPos  = robot.lift.LIFT_SERVO_MAX;
        robot.drive.pose = new Pose2d(AutoPositions.Positions.START_LEFT.position, Math.toRadians(45));
        wristPos = robot.intake.WRIST_INIT;
        clawPos = robot.intake.clawPos;
        spinPos = 0.0;
        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start() {
        robot.led.ledTimer.reset();
        robot.intake.start();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // Update based on Gamepads 1 and 2

        if ( theGamepad1.getButtonPressed(TBDGamepad.Button.DPAD_LEFT))
        {
            runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(45)).build());
        }
        else if ( theGamepad1.getButtonPressed(TBDGamepad.Button.DPAD_UP))
        {
            runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(90)).build());
        }
        else if ( theGamepad1.getButtonPressed(TBDGamepad.Button.DPAD_DOWN))
        {
            runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(-90)).build());
        }

//        if(theGamepad1.getButton(TBDGamepad.Button.DPAD_LEFT)){
//            if(!turning) {
//                runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(45)).build());
//                turning = true;
//            }
//        }else if(theGamepad1.getButton(TBDGamepad.Button.DPAD_UP)){
//            if(!turning) {
//                runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(90)).build());
//                turning = true;
//            }
//        }else if(theGamepad1.getButton(TBDGamepad.Button.DPAD_DOWN)){
//            if(!turning) {
//                runningActions.add(robot.drive.actionBuilder(robot.drive.pose).turnTo(Math.toRadians(-90)).build());
//                turning = true;
//            }
//        }else{
//            turning = false;
//        }


        // If the XXXXXXXX button is pressed, strafe to the specimen locaiton.
        if ( false )  // FIX THIS! Need a button press assignment.
        {
            //robot.alignToSpecimen(0.6);
        }
        else {
            // the alignToSpecimen method changes the LED color to notify the driver that
            // the robot is done aligning to the specimen.  Don't want the intake to override
            // the LED color, so put the LED to the intake's preference when the
            // alignToSpecimen method is not active.
            //robot.led.setToColor(robot.intake.getCalculatedColor());
        }

        // Grab drive commands from Gamepad1
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn  = theGamepad1.getRightX();
        double slow = 0.7;
        double armSlow = 1;

        // Update the mechanisms for the current loop iteration
        robot.intake.update();
        robot.lift.update();
        robot.led.setToColor(robot.intake.getCalculatedColor());

//        if(theGamepad1.getTrigger(LEFT_TRIGGER)>0.1)
        if(theGamepad1.getTriggerBoolean(LEFT_TRIGGER)) {
            slow = 1.0;
            theGamepad1.blipDriver();
        }
//        if(theGamepad1.getTrigger(TBDGamepad.Trigger.RIGHT_TRIGGER)>0.1)
        if(theGamepad1.getTriggerBoolean(RIGHT_TRIGGER)) {
            slow = 0.3;
        }
//        if(theGamepad2.getTrigger(LEFT_TRIGGER)>0.1)
        if(theGamepad2.getTriggerBoolean(LEFT_TRIGGER)) {
            armSlow = 0.4;
        }

        if(theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON)){
            robot.intake.overRideArmPos(true);
        }else{
            robot.intake.overRideArmPos(false);;
        }

        // Lift Controls
        if(theGamepad1.getButton(TBDGamepad.Button.Y)){
            robot.lift.moveToTop();
            liftPower = 0;
        }else if(theGamepad1.getButton(TBDGamepad.Button.B)){
            robot.lift.moveToHanging();
        }else if(theGamepad1.getButton(TBDGamepad.Button.LEFT_BUMPER)){
            robot.lift.lift_target = 0;
        }else if(theGamepad1.getButton(TBDGamepad.Button.RIGHT_BUMPER)){
            robot.lift.lift_target = robot.lift.LIFT_MAX;
        }else if(theGamepad1.getButton(TBDGamepad.Button.A)){
            robot.lift.moveToMin();
            liftPower = 0;
        }

        // Arm controls
        clawPos = robot.intake.clawPos;
        if(theGamepad2.getTrigger(TBDGamepad.Trigger.RIGHT_TRIGGER) > 0.1){
            if(theGamepad2.getButton(TBDGamepad.Button.Y)){
                robot.intake.elbowUp(1.5*armSlow);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.A)){
                robot.intake.elbowDown(1.5*armSlow);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)){
                robot.intake.clawMove(0);
            }
            else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)){
                robot.intake.clawMove(Intake.CLAW_MAX);
            }
            else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_DOWN)) {
                robot.intake.preset(Intake.Positions.INTAKE_SPECIMEN);
                wristPos = Intake.Positions.INTAKE_SPECIMEN.wristPos;
            }
        } else {
            if(gamepad2.dpad_up){
                wristPos = robot.intake.wristPos;
                wristPos -= 0.05*armSlow;
            }
            else if(gamepad2.dpad_down){
                wristPos = robot.intake.wristPos;
                wristPos += 0.05*armSlow;
            }

            if (theGamepad2.getButton(TBDGamepad.Button.Y)) {
                robot.intake.preset(Intake.Positions.HIGH_BASKET);
                wristPos = Intake.Positions.HIGH_BASKET.wristPos;
            } else if (theGamepad2.getButton(TBDGamepad.Button.A)) {
                robot.intake.preset(Intake.Positions.READY_TO_INTAKE);
                wristPos = Intake.Positions.READY_TO_INTAKE.wristPos;
            } else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)) {
                robot.intake.preset(Intake.Positions.LOW_BASKET);
                wristPos = Intake.Positions.LOW_BASKET.wristPos;
            } else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)) {
                robot.intake.preset(Intake.Positions.HIGH_CHAMBER_SCORING);
                wristPos = Intake.Positions.HIGH_CHAMBER_SCORING.wristPos;
            }
        }
        // Manual Arm controls
        if(theGamepad2.getButton(TBDGamepad.Button.X)){
            robot.intake.armUp(0.4*armSlow);
        }
        else if(theGamepad2.getButton(TBDGamepad.Button.B)){
            robot.intake.armDown(-0.8*(armSlow*1.5));
        }
        else{
            if(robot.intake.armTo == 0){
                robot.intake.armStop();
            }
        }

        // Intake boot wheel spinner controls
        if(theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER)){
            robot.intake.spin(1);
            telemetry.addData("intaking",0);
        } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
            robot.intake.spin(-0.25);
            telemetry.addData("outaking",0);
        }else{
            robot.intake.spinStop();
        }

        wristPos = Range.clip(wristPos, robot.intake.WRIST_MIN, robot.intake.WRIST_MAX);
        robot.intake.wristMove(wristPos);

        // Send calculated power to wheels
        if (!turning){
            robot.joystickDrive(forward, strafe, turn * 0.8 * slow, slow);
        }

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if(action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        telemetry.addData("left lift servo: ",robot.lift.getLeftServoPos());
        telemetry.addData("right lift servo: ",robot.lift.getRightServoPos());
        telemetry.addData("left motor position: ", robot.lift.getLiftPosL());
        telemetry.addData("right motor position: ", robot.lift.getLiftPosR());
        telemetry.addData("lift target position: ", robot.lift.lift_target);

        dash.sendTelemetryPacket(packet);
    }
}
