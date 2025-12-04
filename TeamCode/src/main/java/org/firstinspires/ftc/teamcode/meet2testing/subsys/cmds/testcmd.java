package org.firstinspires.ftc.teamcode.meet2testing.subsys.cmds;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.meet2testing.subsys.test;

import java.util.concurrent.TimeUnit;

public class testcmd extends CommandBase
{

    private final test Test;
    private double t;
    private ElapsedTime runtime;

    public testcmd(test subsystem, double g, ElapsedTime r) {
        Test = subsystem;
        t = g;
        runtime = r;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Test);

    }

    @Override
    public void initialize() {
        Test.move(t);
    }

    @Override
    public boolean isFinished()
    {
        if (runtime.seconds() > 2) {
            runtime.reset();
            return true;
        }
        else return false;
    }

}
