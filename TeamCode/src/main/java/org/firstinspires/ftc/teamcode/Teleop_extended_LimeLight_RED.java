package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Teleop")
public class Teleop_extended_LimeLight_RED extends Teleop_BASE
{
    @Override
    public void init() {
        super.init();
        useDeadlines = true;
        isBlue = false;
    }
}
