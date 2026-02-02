package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.d;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.f;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.i;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.p;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants;

import org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.*;
import org.firstinspires.ftc.teamcode.teleop.DriveMeet2;

@TeleOp (name = "Flywheel PIDF Tuning")
@Configurable
public class FlywheelPIDFTuning extends OpMode {

    PIDFController pidf;

    public double lowV = 900;
    public double highV = 2000;

    private DcMotorEx launcher1;
    private DcMotorEx launcher;

    private double vel = 1500;

    public void init() {
//        pidf = new PIDFController(p, i, d, f);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcher1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet2.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    public void loop() {
//        pidf.setPIDF(p, i, d, f);

//        double power = pidf.calculate(launcher.getVelocity(), vel);
//        double power1 = pidf.calculate(launcher1.getVelocity(), vel);

//        if (gamepad1.a) {
//            launcher.setPower(power);
//            launcher1.setPower(power1);
//        } else {
//            launcher.setPower(0);
//            launcher1.setPower(0);
//        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcher1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        if (gamepad1.a) {
            launcher.setVelocity(vel);
            launcher1.setVelocity(vel);
        } else {
            launcher.setPower(0);
            launcher1.setPower(0);
        }

        telemetry.addData("Error", launcher.getVelocity() - vel);
        telemetry.addData("Error 1", launcher1.getVelocity() - vel);
        telemetry.addLine();
        telemetry.addData("Expected Velocity", vel);
        telemetry.addData("Motor 1 Vel", launcher.getVelocity());
        telemetry.addData("Motor 2 Vel", launcher1.getVelocity());
        telemetry.addLine();
        telemetry.addData("Motor 1 Pow", launcher.getPower());
        telemetry.addData("Motor 2 Pow", launcher1.getPower());
        telemetry.addData("P", p);
        telemetry.addData("P", i);
        telemetry.addData("P", d);
        telemetry.addData("P", f);
    }
}
