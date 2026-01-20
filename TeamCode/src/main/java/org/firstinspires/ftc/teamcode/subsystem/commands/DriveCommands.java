package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;

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
            intake.setPower(0);
        }

        public boolean isFinished() {return true;}
    }

    public class intakeOn extends CommandBase
    {
        @Override
        public void initialize(){
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(1);
        }

        public boolean isFinished() {return true;}
    }

    public class toggleIntake extends CommandBase
    {
        @Override
        public void initialize(){
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            if (intake.getPower() == 0) intake.setPower(1);
            else intake.setPower(0);
        }

        public boolean isFinished() {return true;}
    }

    public class intakeIdle extends CommandBase {
        @Override
        public void initialize() {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(0.3);
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
            intake.setPower(1);
            timer.reset();
        }



        @Override
        public boolean isFinished() {return timer.seconds() > 0.2;}
    }

    public class intakeForward extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(1);
            timer.reset();
        }



        @Override
        public boolean isFinished() {return timer.seconds() > 0.1;}
    }

//    public GoBildaPinpointDriver getOdo() {
//        return odo;
//    }

//    public void setOdo(double x, double y) {
//        odo.setPosX(x, DistanceUnit.MM);
//        odo.setPosY(y, DistanceUnit.MM);
//    }

}
