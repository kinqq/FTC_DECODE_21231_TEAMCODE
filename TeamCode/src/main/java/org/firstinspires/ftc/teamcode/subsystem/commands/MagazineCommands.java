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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.*;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

public class MagazineCommands {
    public CRServoImplEx indexer;
    public ServoImplEx hammer;

    public final AnalogInput analog;
    public final DcMotorEx encoder;

    public RevColorSensorV3 bob;
    public RevColorSensorV3 gary;
    public RevColorSensorV3 joe;

    PwmRange fullPwm = new PwmRange(500, 2500);
    double SEC_PER_ROTATION = 0.8325;

    private int target;

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
        PIDController pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        int currentWorldTicks = encoder.getCurrentPosition();
        int targetWorldTicks = target;

        double output = pid.calculate(currentWorldTicks, targetWorldTicks);
        indexer.setPower(Range.clip(output, -1.0, 1.0));
    }

    public boolean updateDone() {
        return encoder.getCurrentPosition() <= target + 100 && encoder.getCurrentPosition() >= target - 100;
    }

    public class zero extends CommandBase{

        public zero() {}

        @Override
        public void initialize() {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public class setSlot extends CommandBase {

        Slot slot;

        public setSlot(Slot slot) {
            this.slot = slot;
        }


        public void initialize() {
            switch (slot) {
                case SECOND: target += 1333; break;
                case THIRD: target -= 1333; break;
            }
        }

        @Override
        public boolean isFinished() {
            return ;
        }
    }

    public void clearAllSlotColors () {
        slotColors.replace(Slot.FIRST, DetectedColor.UNKNOWN);
        slotColors.replace(Slot.SECOND, DetectedColor.UNKNOWN);
        slotColors.replace(Slot.THIRD, DetectedColor.UNKNOWN);
    }

    public class SpinToIntake extends CommandBase{
        Slot slot;
        public SpinToIntake(Slot slot) {this.slot = slot;}

        @Override
        public void initialize() {
            new setSlot(slot);

        }
    }

    public class switchMode extends CommandBase {

        @Override
        public void initialize() {
            target -= 667;
            final Map<Slot, DetectedColor> slotColorsOld = new EnumMap<>(Slot.class);
            slotColorsOld.put(Slot.FIRST, slotColors.get(Slot.FIRST));
            slotColorsOld.put(Slot.SECOND, slotColors.get(Slot.SECOND));
            slotColorsOld.put(Slot.THIRD, slotColors.get(Slot.THIRD));

            slotColors.replace(Slot.FIRST, slotColorsOld.get(Slot.THIRD));
            slotColors.replace(Slot.SECOND, slotColorsOld.get(Slot.FIRST));
            slotColors.replace(Slot.THIRD, slotColorsOld.get(Slot.SECOND));
        }

        public boolean isFinished() {return true;}

    }

    public class hammerUp extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            hammer.setPosition(0.7);
            timer.reset();
        }

        public boolean isFinished() {
            return timer.seconds() > 2;
        }
    }

    public class hammerDown extends CommandBase {
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize() {
            hammer.setPosition(0.48);
            timer.reset();
        }

        public boolean isFinished() {
            return timer.seconds() > 2;
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
                if (H < 167 && H > 150 && S > 0.5 && S < 0.73)
                    slotColors.replace(Slot.THIRD, DetectedColor.GREEN);
                else if (H > 168 && H < 220 && S > 0.2 && S < 0.53)
                    slotColors.replace(Slot.THIRD, DetectedColor.PURPLE);
                else
                    slotColors.replace(Slot.THIRD, DetectedColor.UNKNOWN);
            } else if (bob_gary_joe == 0) {
                if (H < 163 && H > 153 && S > 0.57 && S < 0.9)
                    slotColors.replace(Slot.FIRST, DetectedColor.GREEN);
                else if (H > 197 && H < 238 && S > 0.30 && S < 0.5)
                    slotColors.replace(Slot.FIRST, DetectedColor.PURPLE);                    //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
                else
                    slotColors.replace(Slot.FIRST, DetectedColor.UNKNOWN);
            } else if (bob_gary_joe == 2) {
                if (H < 160 && H > 143 && S > 0.5 && S < 0.66)
                    slotColors.replace(Slot.SECOND, DetectedColor.GREEN);
                else if (H > 163 && H < 200 && S > 0.27 && S < 0.46)
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

    public void stop() {indexer.setPower(0);}
}
