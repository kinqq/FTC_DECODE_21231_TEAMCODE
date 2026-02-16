package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;

import java.util.EnumMap;
import java.util.Map;


public class IndexerCommands {
    private final ServoImplEx indexerPrimary;
    private final ServoImplEx indexerSecondary;
    private final AnalogInput encoder;
    private final NormalizedColorSensor colorSensor;
    private final RevColorSensorV3 rev;

    private final double POSITION_OFFSET;
    private final Map<Slot, DetectedColor> indexerContents = new EnumMap<>(Slot.class);

    private double indexerPosition;
    private double targetAngle;

    private boolean lock;
    private double lockedPosition;

    private String lastPickLog;

    private Slot intakeSlot;

    private boolean firstRev;

    public IndexerCommands(HardwareMap hwMap)
    {
        indexerPrimary = hwMap.get(ServoImplEx.class, "turntable");
        indexerSecondary = hwMap.get(ServoImplEx.class, "turntable1");

        encoder = hwMap.get(AnalogInput.class, "analog");

        colorSensor = hwMap.get(RevColorSensorV3.class, "color");
        rev = hwMap.get(RevColorSensorV3.class, "color");

        indexerPrimary.setPwmRange(new PwmControl.PwmRange(500, 2500));
        indexerSecondary.setPwmRange(new PwmControl.PwmRange(500, 2500));

        POSITION_OFFSET = 0.518;
        for (Slot s : Slot.values()) {
            indexerContents.put(s, DetectedColor.UNKNOWN);
        }

        indexerPosition = 0.1;
        targetAngle = 0.0;

        lock = false;
        lockedPosition = 0.0;

        lastPickLog = "";

        intakeSlot = Slot.FIRST;

        firstRev = true;
    }

    public void start()
    {
        indexerPrimary.setPosition(0.1);
        indexerSecondary.setPosition(0.1);
    }

    public void update()
    {
        if (lock) indexerPosition = lockedPosition;
        else
        {
            double currentPosition = indexerPrimary.getPosition();
            double newPositionPrimary;
            if (intakeSlot == Slot.FIRST) newPositionPrimary = 0.1;
            else if (intakeSlot == Slot.SECOND) newPositionPrimary = 0.273;
            else newPositionPrimary = 0.445;
            double newPositionSecondary = newPositionPrimary + POSITION_OFFSET;

            if (Math.abs(currentPosition - newPositionPrimary) < Math.abs(currentPosition - newPositionSecondary))
            {
                indexerPosition = newPositionPrimary;
                firstRev = true;
            }
            else
            {
                indexerPosition = newPositionSecondary;
                firstRev = false;
            }
        }

        indexerPosition = Range.clip(indexerPosition, 0, 1);
        indexerPrimary.setPosition(indexerPosition);
        indexerSecondary.setPosition(indexerPosition);
    }

    public void nextSlot()
    {
        if (intakeSlot == Slot.FIRST) intakeSlot = Slot.SECOND;
        else if (intakeSlot == Slot.SECOND) intakeSlot = Slot.THIRD;
        else if (intakeSlot == Slot.THIRD) intakeSlot = Slot.FIRST;
    }

    public void prevSlot()
    {
        if (intakeSlot == Slot.FIRST) intakeSlot = Slot.THIRD;
        else if (intakeSlot == Slot.SECOND) intakeSlot = Slot.FIRST;
        else if (intakeSlot == Slot.THIRD) intakeSlot = Slot.SECOND;
    }

    public void setSlot(Slot slot)
    {
        intakeSlot = slot;
    }

    public void lockPosition(double position)
    {
        lock = true;
        lockedPosition = position;
    }

    public void unlockPosition()
    {
        lock = false;
    }

    public void overrideSlot(Slot slot, DetectedColor color)
    {
        indexerContents.replace(slot, color);
    }

    public void overrideActiveSlot(DetectedColor color)
    {
        indexerContents.replace(intakeSlot, color);
    }

    public void setContents(DetectedColor colorOne, DetectedColor colorTwo, DetectedColor colorThree)
    {
        indexerContents.replace(Slot.FIRST, colorOne);
        indexerContents.replace(Slot.SECOND, colorTwo);
        indexerContents.replace(Slot.THIRD, colorThree);
    }

    public void clearContents()
    {
        indexerContents.replace(Slot.FIRST, DetectedColor.UNKNOWN);
        indexerContents.replace(Slot.SECOND, DetectedColor.UNKNOWN);
        indexerContents.replace(Slot.THIRD, DetectedColor.UNKNOWN);
    }

    public DetectedColor getDetectedColor()
    {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float sum = color.red + color.green + color.blue;
        if (sum < 1e-6f) {
            indexerContents.replace(intakeSlot, DetectedColor.UNKNOWN);
            return DetectedColor.UNKNOWN;
        }

        float r = color.red / sum;
        float g = color.green / sum;
        float b = color.blue / sum;

        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float s = (max <= 1e-6f) ? 0f : (delta / max);

        float h;
        if (delta <= 1e-6f) {
            h = 0f;
        } else if (max == r) {
            h = 60f * (((g - b) / delta) % 6f);
        } else if (max == g) {
            h = 60f * (((b - r) / delta) + 2f);
        } else {
            h = 60f * (((r - g) / delta) + 4f);
        }
        if (h < 0f) h += 360f;

        double d = rev.getDistance(DistanceUnit.MM);

        if (s < 0.20f) {
            indexerContents.replace(intakeSlot, DetectedColor.UNKNOWN);
            return DetectedColor.UNKNOWN;
        }

        if (h <= 160f && s >= 0.5) {
            indexerContents.replace(intakeSlot, DetectedColor.GREEN);
            return DetectedColor.GREEN;
        } else if (h >= 154) {
            indexerContents.replace(intakeSlot, DetectedColor.PURPLE);
            return DetectedColor.PURPLE;
        } else {
            indexerContents.replace(intakeSlot, DetectedColor.UNKNOWN);
            return DetectedColor.UNKNOWN;
        }
    }

    public boolean isFull()
    {
        boolean isFull = true;
        if (getSlotColor(Slot.FIRST) == DetectedColor.UNKNOWN) isFull = false;
        else if (getSlotColor(Slot.SECOND) == DetectedColor.UNKNOWN) isFull = false;
        else if (getSlotColor(Slot.THIRD) == DetectedColor.UNKNOWN) isFull = false;

        return isFull;
    }

    public void getNextEmptySlot()
    {
        if (getIntakeSlotColor() == DetectedColor.UNKNOWN) setSlot(intakeSlot);
        else if (getSlotColor(Slot.FIRST) == DetectedColor.UNKNOWN)
        {
            setSlot(Slot.FIRST);
        } else if (getSlotColor(Slot.SECOND) == DetectedColor.UNKNOWN)
        {
            setSlot(Slot.SECOND);
        } else if (getSlotColor(Slot.THIRD) == DetectedColor.UNKNOWN)
        {
            setSlot(Slot.THIRD);
        }
    }

    public boolean indexerIsMoving()
    {
        double currPos = lock ? lockedPosition : indexerPosition;
        double encoderAngle = (encoder.getVoltage() / 3.3) * 360.0;

        if (Math.abs(currPos % POSITION_OFFSET - 0.1) < 0.001) targetAngle = 176.94;
        if (Math.abs(currPos % POSITION_OFFSET - 0.273) < 0.001) targetAngle = 52.58;
        if (Math.abs(currPos % POSITION_OFFSET - 0.445) < 0.001) targetAngle = 299.78;

        return (Math.abs(targetAngle - encoderAngle) > 10);
    }

    public boolean isOnFirstRev()
    {
        return firstRev;
    }

    public String getContents()
    {
        String contents;
        contents = indexerContents.get(Slot.FIRST) + ", " + indexerContents.get(Slot.SECOND) + ", " + indexerContents.get(Slot.THIRD);
        return contents;
    }

    public DetectedColor getSlotColor(Slot slot)
    {
        return indexerContents.get(slot);
    }

    public double getIndexerPosition() {return indexerPosition;}

    public double getTargetAngle() {return targetAngle;}

    public Slot getIntakeSlot() {return intakeSlot;}

    public DetectedColor getIntakeSlotColor() {return indexerContents.get(intakeSlot);}

    public class NextSlot extends CommandBase
    {
        @Override
        public void initialize()
        {
            nextSlot();
        }

        @Override
        public boolean isFinished()
        {
            return indexerIsMoving();
        }
    }

    public class PrevSlot extends CommandBase
    {
        @Override
        public void initialize()
        {
            prevSlot();
        }

        @Override
        public boolean isFinished()
        {
            return indexerIsMoving();
        }
    }

    public class SetSlot extends CommandBase
    {
        private final Slot slot;
        private boolean isNew = true;

        public SetSlot(Slot slot) {
            this.slot = slot;
        }

        @Override
        public void initialize()
        {
            if (intakeSlot != slot)
                setSlot(slot);
            else isNew = false;
        }

        @Override
        public boolean isFinished()
        {
            return !isNew || indexerIsMoving();
        }
    }

    public class SetContents extends CommandBase
    {
        private final DetectedColor one, two, three;

        public SetContents(DetectedColor one, DetectedColor two, DetectedColor three)
        {
            this.one = one;
            this.two = two;
            this.three = three;
        }

        @Override
        public void initialize()
        {
            setContents(one, two, three);
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    public class ClearContents extends CommandBase
    {
        @Override
        public void initialize()
        {
            clearContents();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }


}
