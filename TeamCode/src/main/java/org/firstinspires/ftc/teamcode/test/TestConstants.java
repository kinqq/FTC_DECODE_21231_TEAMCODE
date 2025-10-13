package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class TestConstants extends OpMode {
    public void init() {
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public void loop() {
        telemetry.addData("Iap", Constants.INTAKE_ACTIVE_POWER);
        telemetry.update();
    }
}
