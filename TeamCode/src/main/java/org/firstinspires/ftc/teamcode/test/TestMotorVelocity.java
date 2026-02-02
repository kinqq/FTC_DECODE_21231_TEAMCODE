package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.frozenmilk.sinister.loading.Pinned;


@TeleOp(name = "TestMotorVelocity", group = "Test")
@Pinned
@Configurable
@Disabled

public class TestMotorVelocity extends OpMode {
    private DcMotorEx motor;
    public static String motorName = "launcher";

    // Adjustable target position (encoder ticks)
    public static double targetPower = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);

        // Reset encoder and configure motor
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Set target position and enable motor
        motor.setPower(targetPower);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Telemetry feedback
        telemetry.addData("Target Power", targetPower);
        telemetry.addData("Current", motor.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
