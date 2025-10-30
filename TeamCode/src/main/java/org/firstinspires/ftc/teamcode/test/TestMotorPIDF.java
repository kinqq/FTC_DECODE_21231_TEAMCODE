package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import dev.frozenmilk.sinister.loading.Pinned;

@TeleOp(name = "TestMotorPIDF", group = "Test")
@Configurable
@Pinned
public class TestMotorPIDF extends OpMode {
    private DcMotorEx motor;
    private PIDFController controller = new PIDFController(0,0,0,0);

    // Tunable PIDF coefficients
    public static double p = 0.02;
    public static double i = 0.01;
    public static double d = 1e-4;
    public static double f = 0.0;

    // Adjustable target position (encoder ticks)
    public static int targetPosition = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "intake");

        // Reset encoder and configure motor
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Apply initial PIDF coefficients
        controller.setPIDF(p, i, d, f);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update PIDF coefficients dynamically from dashboard/config
        controller.setPIDF(p, i, d, f);

        // Set target position and enable motor
        motor.setPower(controller.calculate(motor.getCurrentPosition(), targetPosition));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Telemetry feedback
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current", motor.getCurrentPosition());
        telemetry.addData("Error", targetPosition - motor.getCurrentPosition());
        telemetry.addData("PIDF", "P:%.4f I:%.4f D:%.4f F:%.4f", p, i, d, f);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
