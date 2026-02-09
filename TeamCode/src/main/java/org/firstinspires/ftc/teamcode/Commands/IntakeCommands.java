package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ColorTest;

public class IntakeCommands
{
    private final DcMotorEx intakeMotor;
    private final ServoImplEx hammerServo;

    public IntakeCommands(HardwareMap hwMap)
    {
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        hammerServo = hwMap.get(ServoImplEx.class, "hammer");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hammerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void start()
    {
        intakeMotor.setPower(1);
        hammerServo.setPosition(0.63);
    }

    public void hammerPassive()
    {
        hammerServo.setPosition(0.63);
    }

    public void hammerActive()
    {
        hammerServo.setPosition(0.45);
    }

    public void intakeOn()
    {
        intakeMotor.setPower(1);
    }

    public void intakeOff()
    {
        intakeMotor.setPower(0);
    }

    public void toggleIntake()
    {
        if (intakeMotor.getPower() > 0.5) intakeMotor.setPower(0);
        else intakeMotor.setPower(1);
    }

    public void intakeReverse()
    {
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intakeForward()
    {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class HammerPassive extends CommandBase
    {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize()
        {
            hammerServo.setPosition(0.63);
            timer.reset();
        }

        @Override
        public boolean isFinished()
        {
            return timer.milliseconds() > 100;
        }
    }

    public class HammerActive extends CommandBase
    {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize()
        {
            hammerServo.setPosition(0.45);
            timer.reset();
        }

        @Override
        public boolean isFinished()
        {
            return timer.milliseconds() > 100;
        }
    }

    public class IntakeOn extends CommandBase
    {
        @Override
        public void initialize()
        {
            intakeOn();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class IntakeOff extends CommandBase
    {
        @Override
        public void initialize()
        {
            intakeOff();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class IntakeToggle extends CommandBase
    {
        @Override
        public void initialize()
        {
            toggleIntake();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class IntakeReverse extends CommandBase
    {
        @Override
        public void initialize()
        {
            intakeReverse();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class IntakeForward extends CommandBase
    {
        @Override
        public void initialize()
        {
            intakeForward();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }
}
