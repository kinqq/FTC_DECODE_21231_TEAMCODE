package org.firstinspires.ftc.teamcode.subsystem.commands;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.*;

import java.util.EnumMap;
import java.util.Map;

public class MagazineCommands {
    public CRServoImplEx indexer;
    public ServoImplEx hammer;

    PIDController pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

    public final AnalogInput analog;
    public final DcMotorEx encoder;

    public RevColorSensorV3 bob;
    public RevColorSensorV3 gary;
    public RevColorSensorV3 joe;

    PwmRange fullPwm = new PwmRange(500, 2500);
    double SEC_PER_ROTATION = 0.8325;

    private int target = 0;

    private final Map<Slot, DetectedColor> slotColors = new EnumMap<>(Slot.class);

    public MagazineCommands(HardwareMap hwMap) {
        indexer = hwMap.get(CRServoImplEx.class, "turntable");
        hammer = hwMap.get(ServoImplEx.class, "hammer");

        analog = hwMap.get(AnalogInput.class, "encoder");
        encoder = hwMap.get(DcMotorEx.class, "encoderDigital");

        bob = hwMap.get(RevColorSensorV3.class, "color");
        gary = hwMap.get(RevColorSensorV3.class, "gary");
        joe = hwMap.get(RevColorSensorV3.class, "joe");


        indexer.setPwmRange(fullPwm);
        hammer.setPwmRange(fullPwm);

        for (Slot s : Slot.values()) {
            slotColors.put(s, DetectedColor.UNKNOWN);
        }
    }

    public void update() {
        pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        int currentWorldTicks = encoder.getCurrentPosition();
        int targetWorldTicks = target;

        double output = pid.calculate(currentWorldTicks, targetWorldTicks);
        int sign;
        if (output > 0)
        {
            sign = 1;
        }
        else
        {
            sign = -1;
            output *= -1;
        }
        output = Math.sqrt(output);
        output *= sign;

        indexer.setPower(Range.clip(output, -1.0, 1.0));
    }

    public boolean updateDone() {
        return Math.abs(target - encoder.getCurrentPosition()) < 200;
    }

    public class zero extends CommandBase{

        public zero() {}

        @Override
        public void initialize() {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            target = 0;
        }

        @Override
        public boolean isFinished() {return true;}
    }



    public class nextSlot extends CommandBase {
        public nextSlot() {}

        @Override
        public void initialize() {
            target += 1333;
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class prevSlot extends CommandBase {
        public prevSlot() {}

        @Override
        public void initialize() {
            target -= 1333;
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public class setSlot extends CommandBase
    {
        ElapsedTime timer = new ElapsedTime();
        boolean zero = false;
        Slot slot;
        boolean shoot;

        public setSlot(Slot slot) {
            this.slot = slot;
            this.shoot = shoot;
        }


        public void initialize() {
            final Map<Slot, DetectedColor> slotColorsOld = new EnumMap<>(Slot.class);
            slotColorsOld.put(Slot.FIRST, slotColors.get(Slot.FIRST));
            slotColorsOld.put(Slot.SECOND, slotColors.get(Slot.SECOND));
            slotColorsOld.put(Slot.THIRD, slotColors.get(Slot.THIRD));

            switch (slot) {
                case SECOND:
                    target += 1333;
                    slotColors.replace(Slot.FIRST, slotColorsOld.get(Slot.SECOND));
                    slotColors.replace(Slot.SECOND, slotColorsOld.get(Slot.THIRD));
                    slotColors.replace(Slot.THIRD, slotColorsOld.get(Slot.FIRST));
                    break;
                case THIRD:
                    target -= 1333;
                    slotColors.replace(Slot.FIRST, slotColorsOld.get(Slot.THIRD));
                    slotColors.replace(Slot.SECOND, slotColorsOld.get(Slot.FIRST));
                    slotColors.replace(Slot.THIRD, slotColorsOld.get(Slot.SECOND));
                    break;
            }
            timer.reset();
        }

        @Override
        public void execute() {
            if (updateDone() && !zero) {zero = true; timer.reset();}
        }

        @Override
        public boolean isFinished() {
            return (updateDone() && zero && timer.seconds() > 0.5) || timer.seconds() > 2;
        }
    }

    public class clearFirst extends CommandBase {
        @Override
        public void initialize() {
            slotColors.replace(Slot.FIRST, DetectedColor.UNKNOWN);
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

        @Override
        public void initialize() {
            target += 667;
            final Map<Slot, DetectedColor> slotColorsOld = new EnumMap<>(Slot.class);
            slotColorsOld.put(Slot.FIRST, slotColors.get(Slot.FIRST));
            slotColorsOld.put(Slot.SECOND, slotColors.get(Slot.SECOND));
            slotColorsOld.put(Slot.THIRD, slotColors.get(Slot.THIRD));

            slotColors.replace(Slot.FIRST, slotColorsOld.get(Slot.THIRD));
            slotColors.replace(Slot.SECOND, slotColorsOld.get(Slot.FIRST));
            slotColors.replace(Slot.THIRD, slotColorsOld.get(Slot.SECOND));

            timer.reset();
        }

        public boolean isFinished() {return updateDone();}

    }

    public class hammerUp extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            hammer.setPosition(0.8);
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
            hammer.setPosition(0.48);
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() > 0.1;
        }
    }

    public class index extends CommandBase {
        RevColorSensorV3 color;
        int bob_gary_joe;

        public index(int sensor) {
            switch (sensor) {
                case 1: color = bob; bob_gary_joe = 0; break;
                case 2: color = gary; bob_gary_joe = 1; break;
                case 3: color = joe; bob_gary_joe = 2; break;
            }
        }

        @Override
        public void initialize() {
            NormalizedRGBA rgba = color.getNormalizedColors(); //Set normalized RGBA to the normalized values of the indicated color sensor


            float r = clamp01(rgba.red), g = clamp01(rgba.green), b = clamp01(rgba.blue);
            float sum = r + g + b; if (sum < 1e-6f) sum = 1e-6f;
            float rc = r / sum, gc = g / sum, bc = b / sum;

            float[] hsv = new float[3];
            Color.RGBToHSV((int)(rc * 255f), (int)(gc * 255f), (int)(bc * 255f), hsv);
            float H = hsv[0];
            float S = hsv[1];
            float V = hsv[2];

            if (bob_gary_joe == 1) {
                if (H < 169 && H > 148 && S > 0.53 && S < 0.77)
                    slotColors.replace(Slot.THIRD, DetectedColor.GREEN);
                else if (H > 179 && H < 223 && S > 0.33 && S < 0.48)
                    slotColors.replace(Slot.THIRD, DetectedColor.PURPLE);
                else
                    slotColors.replace(Slot.THIRD, DetectedColor.UNKNOWN);
            } else if (bob_gary_joe == 0) {
                if (H < 160 && H > 148 && S > 0.49 && S < 0.7)
                    slotColors.replace(Slot.FIRST, DetectedColor.GREEN);
                else if (H > 170 && H < 240 && S > 0.23 && S < 0.35)
                    slotColors.replace(Slot.FIRST, DetectedColor.PURPLE);                    //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
                else
                    slotColors.replace(Slot.FIRST, DetectedColor.UNKNOWN);
            } else if (bob_gary_joe == 2) {
                if (H < 154 && H > 145 && S > 0.55 && S < 0.62)
                    slotColors.replace(Slot.SECOND, DetectedColor.GREEN);
                else if (H > 155 && H < 205 && S > 0.28 && S < 0.51)
                    slotColors.replace(Slot.SECOND, DetectedColor.PURPLE);                    //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
                else
                    slotColors.replace(Slot.SECOND, DetectedColor.UNKNOWN);
            }
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

    public void stop() {indexer.setPower(0);}
}
