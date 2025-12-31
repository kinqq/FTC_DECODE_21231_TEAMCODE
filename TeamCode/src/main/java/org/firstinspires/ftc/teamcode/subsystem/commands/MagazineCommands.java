package org.firstinspires.ftc.teamcode.subsystem.commands;

import android.graphics.Color;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.*;

import java.util.Base64;
import java.util.EnumMap;
import java.util.Map;

public class MagazineCommands {
    public ServoImplEx indexer;
    public ServoImplEx indexer1;
    public ServoImplEx hammer;
    public DcMotorEx intake;

    PIDController pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

    public final DcMotorEx encoder;

    public RevColorSensorV3 bob;

    private double servoPos = 0;
    private int target = 0;
    private double intakePower = 0;

    private Slot activeSlot = Slot.FIRST;
    private final Map<Slot, DetectedColor> slotColors = new EnumMap<>(Slot.class);

    public MagazineCommands(HardwareMap hwMap) {
        indexer = hwMap.get(ServoImplEx.class, "turntable");
        indexer1 = hwMap.get(ServoImplEx.class, "turntable1");

        hammer = hwMap.get(ServoImplEx.class, "hammer");

        encoder = hwMap.get(DcMotorEx.class, "leftFront");

        intake = hwMap.get(DcMotorEx.class, "intake");

        bob = hwMap.get(RevColorSensorV3.class, "color");

        indexer.scaleRange(0.16, 0.54);
        indexer1.scaleRange(0.53, 0.92);

        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.UNKNOWN);
        }
    }

    public void update() {
        if (servoPos == 0)
        {
            activeSlot = Slot.FIRST;
            target = 0;
        }
        if (servoPos == 0.5)
        {
            activeSlot = Slot.SECOND;
            target = 1285;
        }
        if (servoPos == 1)
        {
            activeSlot = Slot.THIRD;
            target = 2650;
        }

        indexer.setPosition(servoPos);
        indexer1.setPosition(servoPos);
    }

    public boolean updateDone(boolean keep) {
        if (keep && Math.abs(target - encoder.getCurrentPosition()) < 150)
        {
            new hammerDown().initialize();
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(1);
        }
        return Math.abs(target - encoder.getCurrentPosition()) < 150;

    }

    public class zero extends CommandBase{
        boolean reset;

        public zero() {reset = false;}
        public zero(boolean reset) {this.reset = reset;}

        @Override
        public void initialize() {
            if (reset) {
                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            target = 0;
            servoPos = 0;
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class nextSlot extends CommandBase {
        public nextSlot() {}

        @Override
        public void initialize() {
            servoPos += 0.5;
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class prevSlot extends CommandBase {
        public prevSlot() {}

        @Override
        public void initialize() {
            servoPos -= 0.5;
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class setSlot extends CommandBase
    {
        ElapsedTime timer = new ElapsedTime();
        boolean time = false;
        Slot slot;
        boolean keep;

        public setSlot(Slot slot, boolean keep) {
            this.slot = slot;
            this.keep = keep;
        }

        public setSlot(Slot slot) {
            this.slot = slot;
            keep = false;
        }


        public void initialize() {
            double newPos = 0;
            switch (slot) {
                case FIRST:
                    newPos = 0;
                    update();
                    break;
                case SECOND:
                    newPos = 0.5;
                    update();
                    break;
                case THIRD:
                   newPos = 1;
                   update();
                    break;
            }
            if (keep && newPos != servoPos)
            {
                new setHammer(0.48).initialize();
                intakePower = intake.getPower();
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
                time = true;
            }
            servoPos = newPos;
            timer.reset();
        }

        @Override
        public void execute() {
           if (time && timer.seconds() > 0.01) {
               new setHammer(0.49).initialize();
           }
        }

        @Override
        public boolean isFinished() {
            return updateDone(keep);// && zero && timer.seconds() > 0.5) || timer.seconds() > 2;
        }
    }

    public class clearFirst extends CommandBase {
        @Override
        public void initialize() {
            slotColors.replace(activeSlot, DetectedColor.UNKNOWN);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public void clearAllSlotColors () {
        slotColors.replace(Slot.FIRST, DetectedColor.UNKNOWN);
        slotColors.replace(Slot.SECOND, DetectedColor.UNKNOWN);
        slotColors.replace(Slot.THIRD, DetectedColor.UNKNOWN);
    }

    public class switchMode extends CommandBase {
        ElapsedTime timer = new ElapsedTime();

        public switchMode() {

        }

        @Override
        public void initialize() {
            servoPos -= 0.25;
            timer.reset();
        }

        public boolean isFinished() {
            return updateDone(true);
        }

    }

    public void setActive(DetectedColor override) {
        slotColors.replace(Slot.FIRST, override);
    }

    public class setHammer extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();

        double pos;

        public setHammer(double pos) {
            this.pos = pos;
        }

        @Override
        public void initialize() {
            hammer.setPosition(pos);
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > 0.1;
        }
    }

    public class hammerUp extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            hammer.setPosition(0.5);
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > 0.1;
        }
    }

    public class hammerDown extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            hammer.setPosition(0.7);
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > 0.1;
        }
    }

    public class SetSlotColors extends CommandBase{
       DetectedColor FIRST, SECOND, THIRD = DetectedColor.UNKNOWN;

        public SetSlotColors(DetectedColor FIRST, DetectedColor SECOND, DetectedColor THIRD) {
            this.FIRST = FIRST;
            this.SECOND = FIRST;
            this.THIRD = FIRST;
        }

        @Override
        public void initialize() {
            slotColors.replace(Slot.FIRST, FIRST);
            slotColors.replace(Slot.SECOND, SECOND);
            slotColors.replace(Slot.THIRD, THIRD);
        }

        @Override public boolean isFinished() { return true; }
    }

    public class index extends CommandBase {
        RevColorSensorV3 color;
        int bob_gary_joe;

        ElapsedTime timer = new ElapsedTime();
        boolean newBall = false;

        public index(int sensor) {
            switch (sensor) {
                case 1: color = bob; bob_gary_joe = 0; break;
            }
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

            if (H < 160 && H > 148 && S > 0.49 && S < 0.7)
            {
                slotColors.replace(activeSlot, DetectedColor.GREEN);
                timer.reset();
                newBall = true;
            }
            else if (H > 160 && H < 193 && S > 0.35 && S < 0.45)
            {
                slotColors.replace(activeSlot, DetectedColor.PURPLE);                    //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
                timer.reset();
                newBall = true;
            }
            else slotColors.replace(activeSlot, DetectedColor.UNKNOWN);
            }

        @Override
        public boolean isFinished() {
            return !newBall || timer.seconds() > 0;
        }

        private int slotToNum(Slot slot) {
            switch (slot) {
                case FIRST: return 1;
                case SECOND: return 2;
                case THIRD: return 3;
            }
            return -1;
        }

        private Slot numToSlot(int num) {
            switch (num) {
                case 1: return Slot.FIRST;
                case 2: return Slot.SECOND;
                case 3: return Slot.THIRD;
            }
            return Slot.FIRST;
        }

        private float clamp01(float v) {
            return v < 0f ? 0f : (v > 1f ? 1f : v);
        }
    }

    public DetectedColor getSlot(Slot slot) {
        return slotColors.get(slot);
    }

    public int getTarget() {
        return target;
    }
    public int getPos() {
        return encoder.getCurrentPosition();
    }
}
