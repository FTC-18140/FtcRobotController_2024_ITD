package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
@TeleOp
public class NathanPID extends OpMode {
    private PIDController controller;

    public static double p = 0.003, i = 0, d = 0.0001;
    public static double f = 0.1;
    public static double factor_p_down = 0.45;
    public static double factor_d_down = 1.4;

    public static int target = 0;

    private final double ticks_in_degree = 18.182;


    private DcMotorEx arm_motor;



    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor =hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop() {
        controller.setPID(p, i,d);
        int armpos = arm_motor.getCurrentPosition();
        double target_ticks = target * ticks_in_degree;
        if (target_ticks < armpos) {
            controller.setPID(p * factor_p_down * Math.cos(Math.toRadians(clip(armpos/ticks_in_degree,0,180))), i,d * factor_d_down * Math.cos(Math.toRadians(clip(armpos/ticks_in_degree,0,180))));
        }
        double pid = controller.calculate(armpos, clip(target_ticks,0,3000));
        double ff = Math.cos(Math.toRadians(clip(armpos/ticks_in_degree,0,180))) * f;

        double power = pid + ff;
        arm_motor.setPower(power);

        telemetry.addData("pos", armpos / ticks_in_degree);
        telemetry.addData("target", target);
        telemetry.addData("ff", ff);
        telemetry.addData("ticks",target_ticks);
        telemetry.addData("pid", pid);
        telemetry.addData("power", power);
        telemetry.update();

    }
}
