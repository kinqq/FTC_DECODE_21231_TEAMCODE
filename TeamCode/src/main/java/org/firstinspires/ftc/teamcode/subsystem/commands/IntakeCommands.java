package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

public class IntakeCommands {
    public DcMotorEx intake;

    public IntakeCommands(HardwareMap hwMap) {
        this.intake = hwMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private class IntakeOn extends CommandBase {
        private final double p;
        public IntakeOn(double p) { this.p = p; }
        @Override public void execute() { intake.setPower(p); }
        @Override public boolean isFinished() { return true; }
    }

    private class IntakeOff extends CommandBase {
        @Override public void execute() { intake.setPower(0.0); }
        @Override public boolean isFinished() { return true; }
    }

    public CommandBase intakeOn(double power) { return new IntakeOn(power); }
    public CommandBase intakeOff() { return new IntakeOff(); }
}
