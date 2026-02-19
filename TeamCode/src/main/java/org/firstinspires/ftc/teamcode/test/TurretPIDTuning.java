package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.constant.TurretPIDConstants.d;
import static org.firstinspires.ftc.teamcode.constant.TurretPIDConstants.i;
import static org.firstinspires.ftc.teamcode.constant.TurretPIDConstants.p;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

@TeleOp (name = "Turret PID Tuning", group = "Tuning")
public class TurretPIDTuning extends OpMode
{
    DcMotorEx turret;
    double angle = 0.0;
    PIDController pid;

    @Override
    public void init()
    {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(p, i, d);
    }

    @Override
    public void loop()
    {
        pid.setPID(p, i, d);

        int targetReal = (int) Math.round((angle * 5.6111111111) * 384.5 / 360.0);
        double power = pid.calculate(turret.getCurrentPosition(), targetReal);
        turret.setPower(power);

        if (gamepad1.aWasPressed()) angle = -170;
        if (gamepad1.bWasPressed()) angle = 170;

        telemetry.addData("Angle", angle);
        telemetry.addData("Expected", targetReal);
        telemetry.addData("Real", turret.getCurrentPosition());
        telemetry.addData("Power", turret.getPower());
        telemetry.update();
    }
}
