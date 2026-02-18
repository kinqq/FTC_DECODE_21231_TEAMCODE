package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;

public class LaunchCommands {
    private final Servo light;
    IndexerCommands indexerCmds;
    TurretCommands turretCmds;
    IntakeCommands intakeCmds;

    public LaunchCommands(HardwareMap hwMap, IndexerCommands indCmds, TurretCommands tCmds, IntakeCommands intCmds)
    {
        light = hwMap.get(Servo.class, "light");
        light.setDirection(Servo.Direction.REVERSE);

        indexerCmds = indCmds;
        turretCmds = tCmds;
        intakeCmds = intCmds;
    }

    public CommandBase shootRapid()
    {
        boolean moveForward = false;
        if (indexerCmds.isOnFirstRev())
        {
            if (indexerCmds.getIntakeSlot() == Slot.FIRST) moveForward = true;
            if (indexerCmds.getIntakeSlot() == Slot.SECOND) moveForward = true;
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        intakeCmds.new HammerPassive(),
                        intakeCmds.new IntakeOn(),
                        turretCmds.new SpinUp(),
                        new SetLight(0.4)
                ),
                new WaitUntilCommand(turretCmds::flywheelAtExpectedSpeed),
                intakeCmds.new HammerActive(),
                new WaitCommand(150),
                !moveForward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(200),
                !moveForward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        indexerCmds.new ClearContents(),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        intakeCmds.new HammerPassive(),
                        turretCmds.new SpinDown()
                )
        );
    }

    public CommandBase shootMotif(DetectedColor[] motifTranslated)
    {
        Slot first;
        Slot upOne;
        Slot downOne;
        boolean forward = false;

        if (indexerCmds.getSlotColor(Slot.FIRST) == motifTranslated[0])
            first = Slot.FIRST;
        else if (indexerCmds.getSlotColor(Slot.SECOND) == motifTranslated[0])
            first = Slot.SECOND;
        else if (indexerCmds.getSlotColor(Slot.THIRD) == motifTranslated[0])
            first = Slot.THIRD;
        else first = Slot.FIRST;

        if (first == Slot.FIRST)
        {
            upOne = Slot.SECOND;
            downOne = Slot.THIRD;
        }
        else if (first == Slot.SECOND)
        {
            upOne = Slot.THIRD;
            downOne = Slot.FIRST;
        }
        else
        {
            upOne = Slot.FIRST;
            downOne= Slot.SECOND;
        }

        if (indexerCmds.getSlotColor(upOne) == motifTranslated[1]) forward = true;
        else forward = indexerCmds.getSlotColor(downOne) != motifTranslated[1];

        if (
                (forward && (first == Slot.SECOND || first == Slot.THIRD) && !indexerCmds.isOnFirstRev())
                        ||
                        (!forward && (first == Slot.FIRST || first == Slot.SECOND) && indexerCmds.isOnFirstRev())
        ) {
            if (indexerCmds.getSlotColor(first) == DetectedColor.GREEN) forward = !forward;
            else
            {
                if (indexerCmds.getSlotColor(Slot.FIRST) == motifTranslated[0] && first != Slot.FIRST)
                    first = Slot.FIRST;
                else if (indexerCmds.getSlotColor(Slot.SECOND) == motifTranslated[0] && first != Slot.SECOND)
                    first = Slot.SECOND;
                else if (indexerCmds.getSlotColor(Slot.THIRD) == motifTranslated[0] && first != Slot.THIRD)
                    first = Slot.THIRD;
                else first = Slot.SECOND;

                if (first == Slot.FIRST)
                {
                    upOne = Slot.SECOND;
                    downOne = Slot.THIRD;
                }
                else if (first == Slot.SECOND)
                {
                    upOne = Slot.THIRD;
                    downOne = Slot.FIRST;
                }
                else
                {
                    upOne = Slot.FIRST;
                    downOne= Slot.SECOND;
                }

                if (indexerCmds.getSlotColor(upOne) == motifTranslated[1]) forward = true;
                else forward = indexerCmds.getSlotColor(upOne) != motifTranslated[1];
            }
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        indexerCmds.new SetSlot(first),
                        intakeCmds.new HammerPassive(),
                        intakeCmds.new IntakeOn(),
                        turretCmds.new SpinUp(),
                        new SetLight(0.4)
                ),
                new WaitUntilCommand(turretCmds::flywheelAtExpectedSpeed),
                intakeCmds.new HammerActive(),
                new WaitCommand(200),
                !forward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(200),
                !forward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        indexerCmds.new ClearContents(),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        intakeCmds.new HammerPassive(),
                        turretCmds.new SpinDown()
                )
        );
    }

    public CommandBase shootColor(DetectedColor color)
    {
        Slot slot;
        if (indexerCmds.getIntakeSlotColor() == color) slot = indexerCmds.getIntakeSlot();
        else if (indexerCmds.getSlotColor(Slot.FIRST) == color) slot = Slot.FIRST;
        else if (indexerCmds.getSlotColor(Slot.SECOND) == color) slot = Slot.SECOND;
        else if (indexerCmds.getSlotColor(Slot.THIRD) == color) slot = Slot.THIRD;
        else return new InstantCommand();

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        indexerCmds.new SetSlot(slot),
                        intakeCmds.new HammerPassive(),
                        intakeCmds.new IntakeOn(),
                        turretCmds.new SpinUp(),
                        new SetLight(0.4)
                ),
                intakeCmds.new HammerActive(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        intakeCmds.new HammerPassive(),
                        turretCmds.new SpinDown()
                )
        );
    }

    public void setLight(double color)
    {
        light.setPosition(color);
    }

    public class SetLight extends CommandBase
    {
        double color;

        public SetLight(double color)
        {
            this.color = color;
        }

        @Override
        public void initialize() {setLight(color);}

        @Override
        public boolean isFinished() {return true;}
    }
}
