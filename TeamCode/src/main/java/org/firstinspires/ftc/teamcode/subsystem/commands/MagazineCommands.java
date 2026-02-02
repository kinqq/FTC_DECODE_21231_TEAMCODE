package org.firstinspires.ftc.teamcode.subsystem.commands;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;

import java.util.EnumMap;
import java.util.Map;

public class MagazineCommands {
    public ServoImplEx indexer;
    public ServoImplEx indexer1;
    public ServoImplEx hammer;
    private AnalogInput encoderDegrees;
    //private

    public final DcMotorEx encoder;

    public RevColorSensorV3 bob;

    private int target = 0;
    private boolean lock = false;
    private double lockedPos = 0.0;
    public void lockTo(double pos) { lock = true; lockedPos = pos; }
    public void unlock() { lock = false; }
    private static final double OFFSET = 0.518;


    private Slot activeSlot = Slot.FIRST;
    private final Map<Slot, DetectedColor> slotColors = new EnumMap<>(Slot.class);

    public static class Target {
        public final Slot slot;
        public final double pos;
        public Target(Slot slot, double pos) { this.slot = slot; this.pos = pos; }
    }

    public MagazineCommands(HardwareMap hwMap) {
        indexer = hwMap.get(ServoImplEx.class, "turntable");
        indexer1 = hwMap.get(ServoImplEx.class, "turntable1");
        hammer = hwMap.get(ServoImplEx.class, "hammer");

        encoder = hwMap.get(DcMotorEx.class, "intake");
        encoderDegrees = hwMap.get(AnalogInput.class, "analog");

        bob = hwMap.get(RevColorSensorV3.class, "color");

        indexer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        indexer1.setPwmRange(new PwmControl.PwmRange(500, 2500));

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.EMPTY);
        }
    }

    public void start() {
        indexer.setPosition(0.1);
        activeSlot = Slot.FIRST;
    }

    public void update() {
        double target;


        if (lock) {
            target = lockedPos;
        }
        else {
            double base =
                activeSlot == Slot.FIRST  ? 0.1   :
                    activeSlot == Slot.SECOND ? 0.273 :
                        0.445;

            double cur = indexer.getPosition();
            double alt = base + OFFSET;
            target = Math.abs(cur - base) < Math.abs(cur - alt) ? base : alt;
        }

        indexer.setPosition(target);
        indexer1.setPosition(target + 0.003); // somehow this works better
    }

    public boolean isBusy() {
        if (indexer.getPosition() <= 0.2) target = 0;
        if (indexer.getPosition() == 0.273) target = 1340;
        if (indexer.getPosition() <= 0.5 && indexer.getPosition() >= 0.4) target = 2590;
        if (indexer.getPosition() == 0.618) target = 3960;
        if (indexer.getPosition() <= 0.8 && indexer.getPosition() >= 0.7) target = 5370;
        if (indexer.getPosition() <= 1 && indexer.getPosition() >= 0.9) target = 6620;

        return Math.abs(target - encoder.getCurrentPosition()) > 100;
    }

    public double getAnalog() {
        double voltage = encoderDegrees.getVoltage();
        double angleDeg = (voltage / 3.3) * 360.0;

        return angleDeg;
    }

    public boolean isFull() {
        return slotColors.get(Slot.FIRST) != DetectedColor.EMPTY
        && slotColors.get(Slot.SECOND) != DetectedColor.EMPTY
        && slotColors.get(Slot.THIRD) != DetectedColor.EMPTY;
    }

    public double realServoPos() {
        return indexer.getPosition();
    }


    public CommandBase zero() {
        return new InstantCommand(() -> {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
    }

    public class NextSlot extends CommandBase {
        @Override
        public void initialize()
        {
            if (activeSlot == Slot.FIRST) activeSlot = Slot.SECOND;
            else if (activeSlot == Slot.SECOND) activeSlot = Slot.THIRD;
            else if (activeSlot == Slot.THIRD) activeSlot = Slot.FIRST;
        }

        @Override
        public boolean isFinished() {
            return !isBusy();
        }

    }

    public class PrevSlot extends CommandBase {
        @Override
        public void initialize()
        {
            if (activeSlot == Slot.FIRST) activeSlot = Slot.THIRD;
            else if (activeSlot == Slot.SECOND) activeSlot = Slot.FIRST;
            else if (activeSlot == Slot.THIRD) activeSlot = Slot.SECOND;
        }

        @Override
        public boolean isFinished() {
            return !isBusy();
        }

    }


    public CommandBase nextSlot() {
        return new InstantCommand(() -> {
            if (activeSlot == Slot.FIRST) activeSlot = Slot.SECOND;
            else if (activeSlot == Slot.SECOND) activeSlot = Slot.THIRD;
            else if (activeSlot == Slot.THIRD) activeSlot = Slot.FIRST;
        });
    }

    public CommandBase prevSlot() {
        return new InstantCommand(() -> {
            if (activeSlot == Slot.FIRST) activeSlot = Slot.THIRD;
            else if (activeSlot == Slot.SECOND) activeSlot = Slot.FIRST;
            else if (activeSlot == Slot.THIRD) activeSlot = Slot.SECOND;
        });
    }

    public class SetSlot extends CommandBase {
        Slot slot;

        public
        SetSlot(Slot slot) {
            this.slot = slot;
        }

        public void initialize() {
            activeSlot = slot;
        }

        @Override
        public boolean isFinished() {
            return !isBusy();
        }
    }

    public CommandBase setSlot(Slot slot) {
        return new SetSlot(slot);
    }

    public CommandBase setColor(Slot slot, DetectedColor color) {
        return new InstantCommand(() -> slotColors.replace(slot, color));
    }

    public class clearAllSlotColors extends CommandBase {

        public void initialize() {
            slotColors.replace(Slot.FIRST, DetectedColor.EMPTY);
            slotColors.replace(Slot.SECOND, DetectedColor.EMPTY);
            slotColors.replace(Slot.THIRD, DetectedColor.EMPTY);
        }

        public boolean isFinished() {return true;}
    }

    public void setActive(DetectedColor override) {
        slotColors.replace(activeSlot, override);
    }

    public class HammerUp extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            hammer.setPosition(0.45);
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > 0.1;
        }
    }

    public CommandBase hammerUp() { return new HammerUp(); }

    public class HammerDown extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            hammer.setPosition(0.65);
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > 0.1;
        }
    }

    public CommandBase hammerDown() { return new HammerDown(); }


    public CommandBase setSlotColors(DetectedColor FIRST, DetectedColor SECOND, DetectedColor THIRD) {
        return new InstantCommand(() -> {
            slotColors.replace(Slot.FIRST, FIRST);
            slotColors.replace(Slot.SECOND, SECOND);
            slotColors.replace(Slot.THIRD, THIRD);
        });
    }

    public class DistanceIndex extends CommandBase {
        RevColorSensorV3 color = bob;

        ElapsedTime timer = new ElapsedTime();
        boolean newBall = false;

        @Override
        public void initialize() {
            if (bob.getDistance(DistanceUnit.MM) < 44)
            {
                setActive(DetectedColor.UNKNOWN);
                newBall = true;
            }
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return !newBall || timer.seconds() > 0.4;
        }

    }

    public class Index extends CommandBase {
        RevColorSensorV3 color;

        ElapsedTime timer = new ElapsedTime();
        boolean newBall = false;

        public Index() {
            color = bob;
        }

        @Override
        public void initialize() {
            NormalizedRGBA rgba = color.getNormalizedColors(); //Set normalized RGBA to the normalized values of the indicated color sensor

            float r = clamp01(rgba.red), g = clamp01(rgba.green), b = clamp01(rgba.blue);
            float sum = r + g + b;
            if (sum < 1e-6f) sum = 1e-6f;
            float rc = r / sum, gc = g / sum, bc = b / sum;

            float[] hsv = new float[3];
            Color.RGBToHSV((int) (rc * 255f), (int) (gc * 255f), (int) (bc * 255f), hsv);

            float H = hsv[0];
            float S = hsv[1];
            float V = hsv[2];

            if (H > 142 && H < 148 && S > 0.45 && S < 0.48)
            {
                slotColors.replace(activeSlot, DetectedColor.EMPTY);
                timer.reset();
                newBall = true;
            }
            else if (H > 142 && H < 150 && S > 0.47 && S < 0.61)
            {
                slotColors.replace(activeSlot, DetectedColor.GREEN);                    //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
                timer.reset();
                newBall = true;
            }
            else if (H > 142 && H < 163 && S > 0.34 && S < 0.4689)
            {
                slotColors.replace(activeSlot, DetectedColor.PURPLE);                    //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
                timer.reset();
                newBall = true;
            }
            else slotColors.replace(activeSlot, DetectedColor.EMPTY);
            }

        @Override
        public boolean isFinished() {
            return !newBall || timer.seconds() > 0.05;
        }

        private float clamp01(float v) {
            return v < 0f ? 0f : (Math.min(v, 1f));
        }
    }

    public DetectedColor getSlot(Slot slot) {
        return slotColors.get(slot);
    }

    public int getTarget() {
        return target;
    }

    public Slot getActiveSlot() {
        return activeSlot;
    }

    public int getPos() {
        return encoder.getCurrentPosition();
    }
    private double baseOf(Slot s) {
        return s == Slot.FIRST ? 0.1 : (s == Slot.SECOND ? 0.273 : 0.445);
    }

    private boolean wrapMove(double from, double to) {
        return Math.abs(from - to) > 0.5; // 너가 말한 "서보 한계로 wrap"을 강하게 컷
    }

    public Target[] pickTargetsForMotif(DetectedColor[] motif) {
        double startPos = indexer.getPosition();
        Slot[] S = Slot.values(); // FIRST, SECOND, THIRD

        Target[] best = null;
        double bestCost = 1e9;

        for (boolean rev : new boolean[]{false, true}) {
            Slot[] order = rev ? new Slot[]{S[2], S[1], S[0]} : new Slot[]{S[0], S[1], S[2]};

            for (int start = 0; start < 3; start++) {
                Slot[] seq = new Slot[]{
                    order[(start) % 3],
                    order[(start + 1) % 3],
                    order[(start + 2) % 3]
                };

                boolean match = true;
                for (int i = 0; i < 3; i++) {
                    if (slotColors.get(seq[i]) != motif[i]) { match = false; break; }
                }
                if (!match) continue;

                for (int mask = 0; mask < 8; mask++) {
                    Target[] cand = new Target[3];
                    double cur = startPos;
                    double cost = 0;
                    boolean ok = true;

                    for (int i = 0; i < 3; i++) {
                        double b = baseOf(seq[i]);
                        double p = ((mask & (1 << i)) == 0) ? b : (b + OFFSET);

                        if (wrapMove(cur, p)) { ok = false; break; }

                        cost += Math.abs(cur - p);
                        cand[i] = new Target(seq[i], p);
                        cur = p;
                    }

                    if (ok && cost < bestCost) {
                        bestCost = cost;
                        best = cand;
                    }
                }
            }
        }

        return best;
    }
}
