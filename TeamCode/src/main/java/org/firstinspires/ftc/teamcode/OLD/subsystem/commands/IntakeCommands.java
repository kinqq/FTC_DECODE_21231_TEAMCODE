package org.firstinspires.ftc.teamcode.OLD.subsystem.commands;

import static org.firstinspires.ftc.teamcode.constant.Constants.INTAKE_OFF_POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

public class IntakeCommands {
    public DcMotorEx intake;

    public IntakeCommands(HardwareMap hwMap) {
        this.intake = hwMap.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public CommandBase intakeOn(double power) {
        return new InstantCommand(() -> intake.setPower(power));
    }

    public CommandBase intakeOff() {
        return new InstantCommand(() -> intake.setPower(INTAKE_OFF_POWER));
    }
}
