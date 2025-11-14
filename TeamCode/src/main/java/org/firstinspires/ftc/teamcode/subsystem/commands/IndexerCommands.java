package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Slot;
import org.firstinspires.ftc.teamcode.util.DetectedColor;  // ★ 색 enum 위치에 맞게 수정

import java.util.EnumMap;
import java.util.Map;

public class IndexerCommands {
    public ServoImplEx indexer, hammer;
    PwmRange fullPwm = new PwmRange(500, 2500);
    double SEC_PER_ROTATION = 0.66;

    private final Map<Slot, DetectedColor> slotColors = new EnumMap<>(Slot.class);

    public IndexerCommands(HardwareMap hwMap) {
        indexer = hwMap.get(ServoImplEx.class, "turntable");
        hammer = hwMap.get(ServoImplEx.class, "hammer");

        indexer.setPwmRange(fullPwm);
        hammer.setPwmRange(fullPwm);

        // ★ 초기값: 전부 UNKNOWN
        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.UNKNOWN);
        }
    }

    // --------------------------------------------------
    // 슬롯 컬러 저장/조회/초기화 메서드
    // --------------------------------------------------

    /** 해당 슬롯에 감지된 컬러를 저장 */
    public void setSlotColor(Slot slot, DetectedColor color) {
        if (slot == null || color == null) return;
        slotColors.put(slot, color);
    }

    /** 해당 슬롯에 저장된 컬러를 가져오기 (없으면 UNKNOWN) */
    public DetectedColor getSlotColor(Slot slot) {
        if (slot == null) return DetectedColor.UNKNOWN;
        DetectedColor c = slotColors.get(slot);
        return (c != null) ? c : DetectedColor.UNKNOWN;
    }

    /** 모든 슬롯 컬러를 UNKNOWN으로 리셋 */
    public void clearAllSlotColors() {
        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.UNKNOWN);
        }
    }

    /** 특정 슬롯 컬러만 UNKNOWN으로 리셋 */
    public void clearSlotColor(Slot slot) {
        if (slot == null) return;
        slotColors.put(slot, DetectedColor.UNKNOWN);
    }

    /** 특정 색을 가진 슬롯(들) 중 첫 번째 슬롯 반환 (없으면 null) */
    public Slot findFirstSlotWithColor(DetectedColor color) {
        if (color == null) return null;
        for (Slot s : Slot.values()) {
            if (color.equals(slotColors.get(s))) {
                return s;
            }
        }
        return null;
    }

    // --------------------------------------------------
    // 아래부터는 기존 커맨드들 (원래 코드 그대로)
    // --------------------------------------------------

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
            return timer.seconds() > estimatedSeconds;
        }
    }

    public class HammerUp extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();
        private double estimatedSeconds = 0;

        @Override
        public void initialize() {
            double position = Constants.HAMMER_FIRE;

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
