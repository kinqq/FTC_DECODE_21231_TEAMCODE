package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

public class PTOCommands
{
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;

    private final Servo leftClutch;
    private final Servo rightClutch;

    public PTOCommands(HardwareMap hwMap)
    {
        backLeft = hwMap.get(DcMotor.class, "leftBack");
        backRight = hwMap.get(DcMotor.class, "rightBack");
        frontLeft = hwMap.get(DcMotor.class, "leftFront");
        frontRight = hwMap.get(DcMotor.class, "rightFront");
        leftClutch = hwMap.get(Servo.class, "leftClutch");
        rightClutch = hwMap.get(Servo.class, "rightClutch");

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class EngageClutch extends CommandBase
    {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize()
        {
            leftClutch.setPosition(0.5);
            rightClutch.setPosition(0.5);
            timer.reset();
        }

        @Override
        public boolean isFinished()
        {
            return timer.seconds() > 0.1;
        }
    }

    public class ThrottleBack extends CommandBase
    {
        @Override
        public void initialize()
        {
            backLeft.setPower(1);
            backRight.setPower(1);
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class ThrottleFront extends CommandBase
    {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize()
        {
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            timer.reset();
        }

        @Override
        public boolean isFinished()
        {
            return timer.seconds() > 1;
        }
    }

    public class KillFront extends CommandBase
    {
        @Override
        public void initialize()
        {
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class IdleBack extends CommandBase
    {
        @Override
        public void initialize()
        {
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }


}
