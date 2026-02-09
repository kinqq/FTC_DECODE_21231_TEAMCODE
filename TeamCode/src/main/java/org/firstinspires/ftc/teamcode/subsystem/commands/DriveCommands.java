package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.constant.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

public class DriveCommands {
    public DcMotorEx intake; //Intake motor

    public DriveCommands(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake"); //Init intake

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public class intakeOff extends CommandBase
    {
        @Override
        public void initialize() {
            intake.setPower(INTAKE_OFF_POWER);
        }

        public boolean isFinished() {return true;}
    }

    public class intakeOn extends CommandBase
    {
        @Override
        public void initialize(){
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(INTAKE_ACTIVE_POWER);
        }

        public boolean isFinished() {return true;}
    }

    public class toggleIntake extends CommandBase
    {
        @Override
        public void initialize(){
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            if (intake.getPower() == INTAKE_OFF_POWER) intake.setPower(INTAKE_ACTIVE_POWER);
            else intake.setPower(INTAKE_OFF_POWER);
        }

        public boolean isFinished() {return true;}
    }

    public class intakeIdle extends CommandBase {
        @Override
        public void initialize() {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(INTAKE_IDLE_POWER);
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class intakeReverse extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(INTAKE_ACTIVE_POWER);
            timer.reset();
        }



        @Override
        public boolean isFinished() {return timer.seconds() > INTAKE_REVERSE_PULSE_SEC;}
    }

    public class intakeForward extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(INTAKE_ACTIVE_POWER);
            timer.reset();
        }



        @Override
        public boolean isFinished() {return timer.seconds() > INTAKE_FORWARD_PULSE_SEC;}
    }

//    public GoBildaPinpointDriver getOdo() {
//        return odo;
//    }

//    public void setOdo(double x, double y) {
//        odo.setPosX(x, DistanceUnit.MM);
//        odo.setPosY(y, DistanceUnit.MM);
//    }

}
