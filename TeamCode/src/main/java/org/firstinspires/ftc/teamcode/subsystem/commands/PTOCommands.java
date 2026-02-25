package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftD;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftF;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftI;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftP;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightD;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightF;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightI;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

public class PTOCommands
{
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;

    private final Servo leftClutch;
    private final Servo rightClutch;

    public PTOCommands(HardwareMap hwMap)
    {
        backLeft = hwMap.get(DcMotorEx.class, "leftBack");
        backRight = hwMap.get(DcMotorEx.class, "rightBack");
        frontLeft = hwMap.get(DcMotorEx.class, "leftFront");
        frontRight = hwMap.get(DcMotorEx.class, "rightFront");
        leftClutch = hwMap.get(Servo.class, "leftClutch");
        rightClutch = hwMap.get(Servo.class, "rightClutch");

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        rightClutch.setDirection(Servo.Direction.REVERSE);

        backLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(leftP, leftI, leftD, leftF));
        backRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(rightP, rightI, rightD, rightF));
    }

    public int getLeftPos()
    {
        return backLeft.getCurrentPosition();
    }

    public int getRightPos()
    {
        return backRight.getCurrentPosition();
    }

    public double getLeftVel()
    {
        return backLeft.getVelocity();
    }

    public double getRightVel()
    {
        return backRight.getVelocity();
    }

    public class Zero extends CommandBase
    {
        @Override
        public void initialize()
        {
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class DisengageClutch extends CommandBase
    {
        @Override
        public void initialize()
        {
            leftClutch.setPosition(0.06);
            rightClutch.setPosition(0);
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class EngageClutch extends CommandBase
    {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize()
        {
            leftClutch.setPosition(0.18);
            rightClutch.setPosition(0.11);
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
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setPower(-1);
            backRight.setPower(-1);
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
            frontLeft.setPower(0);
            frontRight.setPower(0);
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

    public class KillBack extends CommandBase
    {
        @Override
        public void initialize()
        {
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class PositionLift extends CommandBase
    {
        @Override
        public void initialize()
        {
            backLeft.setTargetPosition(-6100);
            backRight.setTargetPosition(-6100);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setPower(1);
            backRight.setPower(1);
        }

        @Override
        public void execute()
        {
            backRight.setTargetPosition(-6100);
            backLeft.setTargetPosition(backRight.getCurrentPosition());
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setPower(1);
            backRight.setPower(1);
        }

        @Override
        public boolean isFinished()
        {
            return false;
        }
    }

    public class VelLift extends CommandBase
    {
        @Override
        public void initialize()
        {
            backRight.setTargetPosition(-6100);
            backLeft.setTargetPosition(-6100);
            backLeft.setVelocity(-2200);
            backRight.setVelocity(-2200);
            backLeft.setTargetPositionTolerance(0);
            backRight.setTargetPositionTolerance(0);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        @Override
        public void execute()
        {
            double rightP = Math.abs(backRight.getCurrentPosition());
            double leftP = Math.abs(backLeft.getCurrentPosition());

            if (rightP - leftP > 150) backRight.setVelocity(0);
            else if (leftP - rightP > 150) backLeft.setVelocity(0);
            else if (rightP > leftP) {
                backRight.setVelocity(-2200 + 200);
                backLeft.setVelocity(-2200 - 200);
            }
            else if (leftP > rightP)
            {
                backLeft.setVelocity(-2200 + 200);
                backRight.setVelocity(-2200 - 200);
            }



        }

        @Override
        public boolean isFinished()
        {
            return Math.abs(-6100 - backLeft.getCurrentPosition()) < 50 ;
        }
    }
}
