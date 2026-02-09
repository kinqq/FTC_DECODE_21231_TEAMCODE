package org.firstinspires.ftc.teamcode.OLD.subsystem.commands;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.OLD.constant.Constants;
import org.firstinspires.ftc.teamcode.OLD.constant.ConstantsServo;
import org.firstinspires.ftc.teamcode.OLD.constant.Slot;
import org.firstinspires.ftc.teamcode.OLD.constant.DetectedColor;

import java.util.EnumMap;
import java.util.Map;

public class IndexerCommands {
    public ServoImplEx hammer;
    public CRServoImplEx indexer;

    public final AnalogInput analog;
    public final DcMotorEx encoder;

    PwmRange fullPwm = new PwmRange(500, 2500);

    public static final double TICKS_PER_DEG = 4000.0 / 360.0;
    private static final double ANALOG_ZERO_VOLT = 1.20;

    double indexerTargetDeg = 0;
    double indexerOffsetDeg = 0;

    private final PIDController indexerPid =
        new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

    private final Map<Slot, DetectedColor> slotColors = new EnumMap<>(Slot.class);

    public IndexerCommands(HardwareMap hwMap) {
        indexer = hwMap.get(CRServoImplEx.class, "turntable");
        hammer = hwMap.get(ServoImplEx.class, "hammer");
        analog = hwMap.get(AnalogInput.class, "encoder");
        encoder = hwMap.get(DcMotorEx.class, "encoderDigital");

        indexer.setPwmRange(fullPwm);
        hammer.setPwmRange(fullPwm);

        for (Slot s : Slot.values()) slotColors.put(s, DetectedColor.UNKNOWN);
    }

    public void initializeIndexer() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        indexerOffsetDeg = getAnalogDeg();
        indexerTargetDeg = getCurrentDeg();
    }

    public void update() {
        int currentWorldTicks = encoder.getCurrentPosition()
            + (int) Math.round(indexerOffsetDeg * TICKS_PER_DEG);
        int targetWorldTicks = (int) Math.round(indexerTargetDeg * TICKS_PER_DEG);

        double output = indexerPid.calculate(currentWorldTicks, targetWorldTicks);
        output = Math.signum(output) * Math.sqrt(Math.abs(output));
        indexer.setPower(Range.clip(output, -1.0, 1.0));
    }

    public double getCurrentDeg() {
        double ticks = encoder.getCurrentPosition();
        return ticks / TICKS_PER_DEG + indexerOffsetDeg;
    }

    public double getTargetDeg() { return indexerTargetDeg; }

    public double getAnalogDeg() {
        return ((ANALOG_ZERO_VOLT - analog.getVoltage()) / 3.3) * 360.0;
    }

    public void setIndexerTargetDeg(double deg) {
        indexerTargetDeg = deg;
    }

    public double closestAngle(double baseDeg) {
        double current = getCurrentDeg();
        double k = Math.round((current - baseDeg) / 360.0);
        return baseDeg + 360.0 * k;
    }

    public class SetSlotColor extends CommandBase {
        Slot slot;
        DetectedColor color;
        public SetSlotColor(Slot slot, DetectedColor color) {
            this.slot = slot; this.color = color;
        }
        @Override public void initialize() {
            if (slot != null && color != null) slotColors.put(slot, color);
        }
        @Override public boolean isFinished() { return true; }
    }

    public class SetSlotColors extends CommandBase {
        DetectedColor c1, c2, c3;
        public SetSlotColors(DetectedColor c1, DetectedColor c2, DetectedColor c3) {
            this.c1 = c1; this.c2 = c2; this.c3 = c3;
        }
        @Override public void initialize() {
            new SetSlotColor(Slot.FIRST, c1).initialize();
            new SetSlotColor(Slot.SECOND, c2).initialize();
            new SetSlotColor(Slot.THIRD, c3).initialize();
        }
        @Override public boolean isFinished() { return true; }
    }

    public DetectedColor getSlotColor(Slot slot) {
        if (slot == null) return DetectedColor.UNKNOWN;
        return slotColors.getOrDefault(slot, DetectedColor.UNKNOWN);
    }

    public void clearAllSlotColors() {
        for (Slot s : Slot.values()) slotColors.put(s, DetectedColor.UNKNOWN);
    }

    public void clearSlotColor(Slot slot) {
        if (slot != null) slotColors.put(slot, DetectedColor.UNKNOWN);
    }

    public Slot findFirstSlotWithColor(DetectedColor color) {
        if (color == null) return null;
        for (Slot s : Slot.values())
            if (color.equals(slotColors.get(s))) return s;
        return null;
    }

    public class SpinToIntake extends CommandBase {
        private final Slot slot;
        private final ElapsedTime timer = new ElapsedTime();
        private static final double TOLERANCE_DEG = 5.0;
        private static final double TIMEOUT_SEC = 1.0;

        public SpinToIntake(Slot slot) { this.slot = slot; }

        @Override public void initialize() {
            double base = 0;
            if (slot == Slot.FIRST) base = 0;
            if (slot == Slot.SECOND) base = 120;
            if (slot == Slot.THIRD) base = 240;
            indexerTargetDeg = closestAngle(base);
            timer.reset();
        }

        @Override public boolean isFinished() {
            return Math.abs(getCurrentDeg() - indexerTargetDeg) < TOLERANCE_DEG
                || timer.seconds() > TIMEOUT_SEC;
        }
    }

    public CommandBase spinToIntake(Slot slot) { return new SpinToIntake(slot); }

    public class SpinToShoot extends CommandBase {
        private final Slot slot;
        private final ElapsedTime timer = new ElapsedTime();
        private static final double TOLERANCE_DEG = 5.0;
        private static final double TIMEOUT_SEC = 1.0;

        public SpinToShoot(Slot slot) { this.slot = slot; }

        @Override public void initialize() {
            double base = 0;
            if (slot == Slot.FIRST) base = 180;
            if (slot == Slot.SECOND) base = 300;
            if (slot == Slot.THIRD) base = 60;
            indexerTargetDeg = closestAngle(base);
            timer.reset();
        }

        @Override public boolean isFinished() {
            return Math.abs(getCurrentDeg() - indexerTargetDeg) < TOLERANCE_DEG
                || timer.seconds() > TIMEOUT_SEC;
        }
    }

    public CommandBase spinToShoot(Slot slot) { return new SpinToShoot(slot); }


    public class HammerUp extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();
        private double est = 0;

        @Override public void initialize() {
            double p = Constants.HAMMER_FIRE;
            est = Math.abs(p - hammer.getPosition()) * 0.6;
            hammer.setPosition(p);
            timer.reset();
        }

        @Override public boolean isFinished() { return timer.seconds() >= est; }
    }

    public class HammerDown extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();
        private double est = 0;

        @Override public void initialize() {
            double p = Constants.HAMMER_REST;
            est = Math.abs(p - hammer.getPosition()) * 0.6;
            hammer.setPosition(p);
            timer.reset();
        }

        @Override public boolean isFinished() { return timer.seconds() >= est; }
    }
}
