package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constant.Constants;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;

import java.util.EnumMap;
import java.util.Map;

public class IndexerCommands {
    public ServoImplEx indexer, hammer;
    PwmRange fullPwm = new PwmRange(500, 2500);
    double SEC_PER_ROTATION = 0.8325;

    private final Map<Slot, DetectedColor> slotColors = new EnumMap<>(Slot.class);

    public IndexerCommands(HardwareMap hwMap) {
        indexer = hwMap.get(ServoImplEx.class, "turntable");
        hammer = hwMap.get(ServoImplEx.class, "hammer");

        indexer.setPwmRange(fullPwm);
        hammer.setPwmRange(fullPwm);

        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.UNKNOWN);
        }
    }

    public class SetSlotColor extends CommandBase {
        Slot slot;
        DetectedColor color;
        public SetSlotColor(Slot slot, DetectedColor color) {
            this.slot = slot;
            this.color = color;
        }

        @Override
        public void initialize() {
            if (slot == null || color == null) return;
            slotColors.put(slot, color);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class SetSlotColors extends CommandBase {
        DetectedColor c1, c2, c3;
        public SetSlotColors(DetectedColor c1, DetectedColor c2, DetectedColor c3) {
            this.c1 = c1;
            this.c2 = c2;
            this.c3 = c3;
        }

        @Override
        public void initialize() {
            new SetSlotColor(Slot.FIRST, c1).initialize();
            new SetSlotColor(Slot.SECOND, c2).initialize();
            new SetSlotColor(Slot.THIRD, c3).initialize();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


    public DetectedColor getSlotColor(Slot slot) {
        if (slot == null) return DetectedColor.UNKNOWN;
        DetectedColor c = slotColors.get(slot);
        return (c != null) ? c : DetectedColor.UNKNOWN;
    }


    public void clearAllSlotColors() {
        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.UNKNOWN);
        }
    }


    public void clearSlotColor(Slot slot) {
        if (slot == null) return;
        slotColors.put(slot, DetectedColor.UNKNOWN);
    }


    public Slot findFirstSlotWithColor(DetectedColor color) {
        if (color == null) return null;
        for (Slot s : Slot.values()) {
            if (color.equals(slotColors.get(s))) {
                return s;
            }
        }
        return null;
    }


    public class SpinToIntake extends CommandBase {
        private Slot slot;
        private ElapsedTime timer = new ElapsedTime();
        private double estimatedSeconds = 0;

        public SpinToIntake(Slot slot) {
            this.slot = slot;
        }

        @Override
        public void initialize() {
            double position = 0;
            if (slot == Slot.FIRST) position = Constants.SLOT1_HOLD;
            if (slot == Slot.SECOND) position = Constants.SLOT2_HOLD;
            if (slot == Slot.THIRD) position = Constants.SLOT3_HOLD;

            double change = Math.abs(position - indexer.getPosition());
            estimatedSeconds = change * SEC_PER_ROTATION;

            indexer.setPosition(position);

            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > estimatedSeconds;
        }
    }
    public CommandBase spinToIntake(Slot slot) { return new SpinToIntake(slot); }
    public class SpinToShoot extends CommandBase {
        private Slot slot;
        private ElapsedTime timer = new ElapsedTime();
        private double estimatedSeconds = 0;

        public SpinToShoot(Slot slot) {
            this.slot = slot;
        }

        @Override
        public void initialize() {
            double position = 0;
            if (slot == Slot.FIRST) position = Constants.SLOT1_SHOOT;
            if (slot == Slot.SECOND) position = Constants.SLOT2_SHOOT;
            if (slot == Slot.THIRD) position = Constants.SLOT3_SHOOT;

            double change = Math.abs(position - indexer.getPosition());
            estimatedSeconds = change * SEC_PER_ROTATION;

            indexer.setPosition(position);

            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > estimatedSeconds || timer.seconds() > 2;
        }
    }

    public class HammerUp extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();
        private double estimatedSeconds = 0;

        @Override
        public void initialize() {
            double position = Constants.HAMMER_FIRE;

            double change = Math.abs(position - hammer.getPosition());
            estimatedSeconds = change * SEC_PER_ROTATION + 0.1;

            hammer.setPosition(position);

            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > estimatedSeconds;
        }
    }

//    public class HammerUpAnalog extends CommandBase {
//        double position;
//
//        @Override
//        public void initialize() {
//            position = Constants.HAMMER_FIRE;
//            hammer.setPosition(position);
//        }
//
//        @Override
//        public boolean isFinished() {
//            /*
//                double voltage = hammerAnalog.getVoltage();
//                double currPosition = voltage / 3.3;
//                return Math.abs(currPosition - position) < 0.01 // less than 7.2 deg
//            */
//            return true;
//        }
//    }

    public class HammerDown extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();
        private double estimatedSeconds = 0;

        @Override
        public void initialize() {
            double position = Constants.HAMMER_REST;

            double change = Math.abs(position - hammer.getPosition());
            estimatedSeconds = change * SEC_PER_ROTATION;

            hammer.setPosition(position);

            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > estimatedSeconds;
        }
    }

    public class Next extends CommandBase {
        CommandBase command;
        boolean isValid = true;     // ★ 원래 버그: 기본 false → true로
        Slot nextSlot;

        public Next(Slot currentSlot) {
            if (currentSlot == Slot.FIRST) nextSlot = Slot.SECOND;
            else if (currentSlot == Slot.SECOND) nextSlot = Slot.THIRD;
            else if (currentSlot == Slot.THIRD) nextSlot = Slot.FIRST;
            else isValid = false;
        }

        @Override
        public void initialize() {
            if (isValid) {
                command = new SpinToIntake(nextSlot);
                command.initialize();
            }
        }

        @Override
        public boolean isFinished() {
            if (!isValid) return true;
            return command.isFinished();
        }
    }
}