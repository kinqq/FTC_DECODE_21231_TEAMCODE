package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.constant.Slot;

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

    public void initialize() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        offsetDeg = getAnalogDeg() + 30; // add 30 to compensate for servo offset
        slot = Slot.FIRST;

        setSlotHold();
    }

    // Update
    public void update() {
        pid.setPID(Constants.SERVO_P, Constants.SERVO_I, Constants.SERVO_D);

        int currentWorldTicks =
            encoder.getCurrentPosition() + (int) Math.round(offsetDeg * TICKS_PER_DEG);
        int targetWorldTicks = (int) Math.round(targetDeg * TICKS_PER_DEG);

        double output = pid.calculate(currentWorldTicks, targetWorldTicks);
        magazine.setPower(Range.clip(output, -1.0, 1.0));
    }

    //
    // Public API
    //

    public void setSlotHold() {
        setHoldTargetForCurrentSlot();
    }

    public void setSlotHold(Slot slot) {
        setSlot(slot);
        setSlotHold();
    }

    public void setSlotLaunch() {
        setLaunchTargetForCurrentSlot();
    }

    public void setSlotLaunch(Slot slot) {
        setSlot(slot);
        setSlotLaunch();
    }

    public void nextSlot() {
        switch (slot) {
            case FIRST:  setSlotHold(Slot.SECOND); break;
            case SECOND: setSlotHold(Slot.THIRD);  break;
            case THIRD:  setSlotHold(Slot.FIRST);  break;
        }
    }

    public void prevSlot() {
        switch (slot) {
            case FIRST:  setSlotHold(Slot.THIRD);  break;
            case SECOND: setSlotHold(Slot.FIRST);  break;
            case THIRD:  setSlotHold(Slot.SECOND); break;
        }
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public double getEncoderDeg() {
        return encoder.getCurrentPosition() / TICKS_PER_DEG;
    }

    public double getAnalogDeg() {
        return (analog.getVoltage() / 3.3) * 360.0;
    }

    public double getPower() {
        return magazine.getPower();
    }

    //
    // Internal Math
    //

    private void setSlot(Slot slot) {
        this.slot = slot;
    }

    private double holdDegForSlot(Slot s) {
        switch (s) {
            case FIRST:  return 0.0;
            case SECOND: return 120.0;
            case THIRD:  return 240.0;
        }
        return 0.0;
    }

    private double launchDegForSlot(Slot s) {
        switch (s) {
            case FIRST:  return 180.0;
            case SECOND: return 300.0;
            case THIRD:  return 60.0;
        }
        return 180.0;
    }

    private double getCurrentWorldDeg() {
        double ticks = encoder.getCurrentPosition();
        return ticks / TICKS_PER_DEG + offsetDeg;
    }

    // choose the closest 0/120/240 (+ n*360) to current angle
    private void setHoldTargetForCurrentSlot() {
        double current = getCurrentWorldDeg();
        double base = holdDegForSlot(slot);

        double k = Math.round((current - base) / 360.0);
        targetDeg = base + 360.0 * k;
    }

    // same as above but for launch angles (180/300/60)
    private void setLaunchTargetForCurrentSlot() {
        double current = getCurrentWorldDeg();
        double base = launchDegForSlot(slot);

        double k = Math.round((current - base) / 360.0);
        targetDeg = base + 360.0 * k;
    }
}
