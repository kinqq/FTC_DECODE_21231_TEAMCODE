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

import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public class MagazineCommands {
    public ServoImplEx indexer;
    public ServoImplEx indexer1;
    public ServoImplEx hammer;

    public final DcMotorEx encoder;
    private final AnalogInput analogInput;

    public RevColorSensorV3 bob;

    private double servoPos = 0;
    private double oldPos = 0;
    private double target = 0;
    private boolean lock = false;
    private double lockedPos = 0.0;
    public void unlock() { lock = false; }
    private static final double OFFSET = 0.518;
    private String lastPickLog = "";

    public String getLastPickLog() {
        return lastPickLog;
    }


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
        analogInput = hwMap.get(AnalogInput.class, "analog");

        bob = hwMap.get(RevColorSensorV3.class, "color");

        indexer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        indexer1.setPwmRange(new PwmControl.PwmRange(500, 2500));

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.UNKNOWN);
        }
    }

    public void update() {
        if (lock) {
            servoPos = lockedPos;
        }
        else {
            double base =
                activeSlot == Slot.FIRST ? 0.1 :
                    activeSlot == Slot.SECOND ? 0.273 :
                        0.445;

            double cur = indexer.getPosition();
            double alt = base + OFFSET;
            servoPos = Math.abs(cur - base) < Math.abs(cur - alt) ? base : alt;
        }

        indexer.setPosition(servoPos);
        indexer1.setPosition(servoPos + 0.003); // somehow this works better
    }

    public boolean isBusy() {
        double currPos = lock ? lockedPos : servoPos;

        if (Math.abs(currPos % OFFSET - 0.1) < 0.001) target = 176.94;
        if (Math.abs(currPos % OFFSET - 0.273) < 0.001) target = 52.58;
        if (Math.abs(currPos % OFFSET - 0.445) < 0.001) target = 299.78;

        return Math.abs(target - getAnalogAngle()) > 10;
    }

    public double getAnalogAngle() {
        double voltage = analogInput.getVoltage();
        return (voltage / 3.3) * 360.0;
    }

    public CommandBase zero() {
        return new InstantCommand(() -> {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
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
        ElapsedTime timer = new ElapsedTime();

        public SetSlot(Slot slot) {
            this.slot = slot;
        }

        public void initialize() {
            activeSlot = slot;
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return !isBusy() || timer.seconds() > 2.0;
        }
    }

    public CommandBase setSlot(Slot slot) {
        return new SetSlot(slot);
    }

    public class LockSlot extends CommandBase {
        double position;
        ElapsedTime timer = new ElapsedTime();

        public LockSlot(double position) {
            this.position = position;
        }

        public void initialize() {
            lock = true;
            lockedPos = position;
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return !isBusy() || timer.seconds() > 2.0;
        }
    }

    public CommandBase lockSlot(double pos) {
        return new LockSlot(pos);
    }

    public CommandBase lockSlot(Slot slot) {
        double pos = 0.1;
        if (slot == Slot.FIRST) pos = 0.1;
        if (slot == Slot.SECOND) pos = 0.273;
        if (slot == Slot.THIRD) pos = 0.445;
        return new LockSlot(pos);
    }

    public void resetPos() {servoPos = oldPos;}

    public CommandBase setColor(Slot slot, DetectedColor color) {
        return new InstantCommand(() -> slotColors.replace(slot, color));
    }

    public CommandBase clearAllSlotColors () {
        return new InstantCommand(() -> {
            setColor(Slot.FIRST, DetectedColor.UNKNOWN);
            setColor(Slot.SECOND, DetectedColor.UNKNOWN);
            setColor(Slot.THIRD, DetectedColor.UNKNOWN);
        });
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
            hammer.setPosition(0.63);
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

    public class DistanceSwitcher extends CommandBase {
        RevColorSensorV3 color = bob;

        ElapsedTime timer = new ElapsedTime();
        boolean newBall = false;

        @Override
        public void initialize() {
            if (bob.getDistance(DistanceUnit.MM) < 44)
            {
                setActive(DetectedColor.GREEN);
                newBall = true;
            }
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return !newBall || timer.seconds() > 0.4;
        }

    }

    public class WaitForAnyArtifact extends CommandBase {
        ElapsedTime timer = new ElapsedTime();

        boolean start = false;

        @Override
        public void execute() {
            if (!start) {
                timer.reset();
                start = true;
            }
        }

        @Override
        public boolean isFinished() {
            return bob.getDistance(DistanceUnit.MM) < 44 || timer.seconds() > 3.0;
        }
    }

    public CommandBase waitForAnyArtifact() { return new WaitForAnyArtifact(); }

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

            if (H < 152 && H > 145 && S > 0.40 && S < 0.66)
            {
                slotColors.replace(activeSlot, DetectedColor.GREEN);
                timer.reset();
                newBall = true;
            }
            else if (H < 169 && H > 154 && S > 0.40 && S < 0.53)
            {
                slotColors.replace(activeSlot, DetectedColor.PURPLE);                    //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
                timer.reset();
                newBall = true;
            }
            else slotColors.replace(activeSlot, DetectedColor.UNKNOWN);
            }

        @Override
        public boolean isFinished() {
            return !newBall || timer.seconds() > 0.05;
        }

        private float clamp01(float v) {
            return v < 0f ? 0f : (v > 1f ? 1f : v);
        }
    }

    public DetectedColor getSlot(Slot slot) {
        return slotColors.get(slot);
    }

    public double getTarget() {
        return target;
    }
    public int getPos() {
        return encoder.getCurrentPosition();
    }
    public double getServoPos() {
        return servoPos;
    }

    private String fmtTarget(Target t) {
        if (t == null) return "null";
        Slot s = t.slot;
        double b = baseOf(s);
        int wrap = Math.abs(t.pos - b) < 1e-6 ? 0 : 1;
        return s.name() + "[w" + wrap + "]@" + String.format("%.3f", t.pos);
    }

    private String motifStr(DetectedColor[] m) {
        if (m == null) return "null";
        return m[0] + "," + m[1] + "," + m[2];
    }

    private double baseOf(Slot s) { return s == Slot.FIRST ? 0.1 : (s == Slot.SECOND ? 0.273 : 0.445); }
    public Target[] pickTargetsForMotif(DetectedColor[] motif) {
        if (motif == null || motif.length != 3) return null;

        final double EPS = 1e-6;
        final double startPos = indexer.getPosition(); // [0,1]

        Slot[] S = Slot.values(); // FIRST, SECOND, THIRD

        Target[] best = null;
        double bestCost = 1e18;

        for (boolean rev : new boolean[]{false, true}) {
            Slot[] order = rev
                ? new Slot[]{S[2], S[1], S[0]}
                : new Slot[]{S[0], S[1], S[2]};

            for (int start = 0; start < 3; start++) {
                Slot[] seq = new Slot[]{
                    order[start % 3],
                    order[(start + 1) % 3],
                    order[(start + 2) % 3]
                };

                boolean match = true;
                for (int i = 0; i < 3; i++) {
                    if (slotColors.get(seq[i]) != motif[i]) { match = false; break; }
                }
                if (!match) continue;

                for (int mask = 0; mask < 8; mask++) {
                    double[] p = new double[3];
                    Target[] cand = new Target[3];
                    boolean ok = true;

                    for (int i = 0; i < 3; i++) {
                        double b = baseOf(seq[i]);
                        double pi = ((mask & (1 << i)) == 0) ? b : (b + OFFSET);
                        if (pi < 0.0 || pi > 1.0) { ok = false; break; }
                        p[i] = pi;
                        cand[i] = new Target(seq[i], pi);
                    }
                    if (!ok) continue;

                    boolean forward = (p[0] <= p[1] + EPS) && (p[1] <= p[2] + EPS);
                    boolean backward = (p[0] + EPS >= p[1]) && (p[1] + EPS >= p[2]);
                    if (!forward && !backward) continue;

                    double cost =
                        Math.abs(startPos - p[0]) +
                            Math.abs(p[0] - p[1]) +
                            Math.abs(p[1] - p[2]);

                    if (cost < bestCost) {
                        bestCost = cost;
                        best = cand;
                    }
                }
            }
        }
        if (best == null) {
            lastPickLog =
                "pickTargets: motif=" + motifStr(motif) +
                    " start=" + String.format("%.3f", indexer.getPosition()) +
                    " => BEST=null";
            return null;
        }

        lastPickLog =
            "pickTargets: motif=" + motifStr(motif) +
                " start=" + String.format("%.3f", indexer.getPosition()) +
                " => " + fmtTarget(best[0]) + " -> " + fmtTarget(best[1]) + " -> " + fmtTarget(best[2]);


        return best;
    }



}
