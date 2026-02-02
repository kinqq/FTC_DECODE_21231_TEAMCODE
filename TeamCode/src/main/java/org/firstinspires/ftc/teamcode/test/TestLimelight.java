package org.firstinspires.ftc.teamcode.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;

@TeleOp(group = "Test")

public class TestLimelight extends OpMode {
    LimelightCommands ll;
    LimelightCommands.Motif motif;

    @Override
    public void init() {
        ll = new LimelightCommands(hardwareMap);
        ll.start(0);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void loop() {
        motif = ll.detectMotifFromLatest();

        telemetry.addData("Motif", motif);
        if (ll.getMotifMeasurementFromLatest() != null) {
            telemetry.addData("ID", ll.getMotifMeasurementFromLatest().tagId);
            telemetry.addData("X Deg", ll.getMotifMeasurementFromLatest().xDeg);
            telemetry.addData("Y Deg", ll.getMotifMeasurementFromLatest().yDeg);
        }
        telemetry.update();
    }
}
