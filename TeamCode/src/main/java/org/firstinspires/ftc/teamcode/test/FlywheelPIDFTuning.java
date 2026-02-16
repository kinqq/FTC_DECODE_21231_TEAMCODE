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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PController;

import org.firstinspires.ftc.teamcode.teleop.DriveMeet2;

@TeleOp (name = "Flywheel PIDF Tuning")
@Configurable
public class FlywheelPIDFTuning extends OpMode {

    PController pid;

    public double lowV = 900;
    public double highV = 2000;

    private DcMotorEx launcher1;
    private DcMotorEx launcher;
    private VoltageSensor voltage;

    private double vel = 1500;

    public void init() {
        pid = new PController(p);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet2.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    public void loop() {
        double kF = f;
        pid.setP(p);

        double pidResult = pid.calculate(-launcher1.getVelocity(), vel);
        double ff = kF * vel;

        double power = Range.clip(pidResult + ff, -1, 1);


        if (gamepad1.a) {
            launcher.setPower(power);
            launcher1.setPower(power);
        } else {
            launcher1.setPower(0);
            launcher.setPower(0);
        }

        if (gamepad1.leftBumperWasPressed()) vel -= 100;
        if (gamepad1.rightBumperWasPressed()) vel += 100;

        telemetry.addData("Error 1", (-launcher1.getVelocity()) - vel);
        telemetry.addLine();
        telemetry.addData("Expected Velocity", vel);
        telemetry.addData("Motor 2 Vel", -launcher1.getVelocity());
        telemetry.addLine();
        telemetry.addData("Motor 1 Pow", launcher.getPower());
        telemetry.addData("Motor 2 Pow", launcher1.getPower());
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);
    }
}
