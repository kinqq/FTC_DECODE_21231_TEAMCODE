package org.firstinspires.ftc.teamcode.OLD.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.OLD.constant.Slot;

public class MagazineSubsystem {
    private final CRServoImplEx magazine;
    private final AnalogInput analog;
    private final DcMotorEx encoder;

    private double targetDeg = 0;
    private double offsetDeg = 0;
    private Slot slot = Slot.FIRST;
    public double TICKS_PER_DEG = 4000.0 * 4 / 360;

    private final PIDController pid =
        new PIDController(Constants.SERVO_P, Constants.SERVO_I, Constants.SERVO_D);

    public MagazineSubsystem(HardwareMap hwMap) {
        magazine = hwMap.get(CRServoImplEx.class, "turntable");
        analog = hwMap.get(AnalogInput.class, "encoder");
        encoder = hwMap.get(DcMotorEx.class, "encoderDigital");
    }

    // Update
    public void update() {
        pid.setPID(Constants.SERVO_P, Constants.SERVO_I, Constants.SERVO_D);

        int currentWorldTicks =
            encoder.getCurrentPosition();
        int targetWorldTicks = (int) Math.round(targetDeg);//(int) Math.round(targetDeg * TICKS_PER_DEG);

        double output = pid.calculate(currentWorldTicks, targetWorldTicks);
        magazine.setPower(Range.clip(output, -1.0, 1.0));
    }

    public void nextSlot() {
        switch (slot) {
            case FIRST: slot = Slot.SECOND; break;
            case SECOND: slot = Slot.THIRD; break;
            case THIRD: slot = Slot.FIRST; break;
        }
        targetDeg += 1333;
    }

    public void prevSlot() {
        switch (slot) {
            case FIRST: slot = Slot.THIRD; break;
            case SECOND: slot = Slot.FIRST; break;
            case THIRD: slot = Slot.SECOND; break;
        }
        targetDeg -= 1333;
    }

    public void setSlot(Slot slot) {
        this.slot = slot;
    }

    public Slot getSlot() {
        return this.slot;
    }

    public int getTarget() {
        return (int) targetDeg;
    }


}
