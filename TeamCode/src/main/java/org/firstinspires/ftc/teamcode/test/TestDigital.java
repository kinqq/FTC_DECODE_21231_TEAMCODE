package org.firstinspires.ftc.teamcode.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(group = "Test")
public class TestDigital extends OpMode {
    DigitalChannel digital1, digital2;

    public void init() {
        digital1 = hardwareMap.get(DigitalChannel.class, "digital1");
        digital2 = hardwareMap.get(DigitalChannel.class, "digital2");
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }
    public void loop() {
        telemetry.addData("Digital 1", digital1.getState());
        telemetry.addData("Digital 2", digital2.getState());
        telemetry.addData("STATE", digital1.getState() && digital2.getState());
        telemetry.update();
    }
}
