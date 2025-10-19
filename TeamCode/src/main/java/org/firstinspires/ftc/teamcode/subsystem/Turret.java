package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.ConstantsServo.STATIC_COMP;
import static org.firstinspires.ftc.teamcode.util.ConstantsServo.kD_VEL;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.ConstantsServo;
import org.firstinspires.ftc.teamcode.util.ContinuousAbsoluteTracker;

public class Turret {
    private CRServo turret;
    private Servo launchAngle;
    private ContinuousAbsoluteTracker tracker;
    private PIDController pid;

    private static final double SERVO_TO_TURRET_GEAR_RATIO = 5.6111111111;
    private static final double LATENCY_SEC = 0.06;
    private static final double MAX_PREDICT_DEG = 10.0;
    private static final double STATIC_ANGLE_THRESH = 1.0;

    private double targetTurretDeg = 0.0;
    private double lastCmd = 0.0;
    private double angle = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public Turret(HardwareMap hwMap) {
        turret = hwMap.get(CRServo.class, "turret");
        launchAngle = hwMap.get(Servo.class, "launchAngle");
        AnalogInput analog = hwMap.get(AnalogInput.class, "analog");
        tracker = new ContinuousAbsoluteTracker(analog, 360);
        pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        turret.setPower(0);
        loopTimer.reset();
        tracker.updateAbsolute();
        targetTurretDeg = getAngleDeg();
    }

    /** Main control loop */
    public void update() {
        tracker.updateAbsolute();
        pid.setPID(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        double measuredServoDeg = tracker.getTotalAngleDeg();
        double targetServoDeg = targetTurretDeg * SERVO_TO_TURRET_GEAR_RATIO;

        double velDpsServo = tracker.getEstimatedVelocityDps();
        double errMeasured = targetServoDeg - measuredServoDeg;

        double predictedServoDeg = measuredServoDeg + velDpsServo * LATENCY_SEC;
        predictedServoDeg = measuredServoDeg + Range.clip(
            predictedServoDeg - measuredServoDeg, -MAX_PREDICT_DEG, MAX_PREDICT_DEG);

        double pidOut = pid.calculate(predictedServoDeg, targetServoDeg);
        double velDamping = -kD_VEL * velDpsServo;
        double out = pidOut + velDamping;

        if (Math.abs(out) > 0 && Math.abs(out) < STATIC_COMP &&
            Math.abs(errMeasured) > STATIC_ANGLE_THRESH)
            out = Math.signum(out) * STATIC_COMP;

        lastCmd = Range.clip(out, -1.0, 1.0);
        turret.setPower(lastCmd);

        double anglePos = -0.0038 * angle + 0.871;
        launchAngle.setPosition(anglePos);

        loopTimer.reset();
    }

    /** Set absolute turret angle in degrees */
    public void setTarget(double turretDeg) {
        targetTurretDeg = turretDeg;
    }

    /** Increment target angle */
    public void adjustTarget(double deltaTurretDeg) {
        targetTurretDeg += deltaTurretDeg;
    }

    /** Zero current turret position */
    public void zeroHere() {
        tracker.rebaseAbsolute(0.0);
        targetTurretDeg = 0.0;
        pid.reset();
    }

    /** Launch angle range from 20 deg to 45 deg  */
    public void setLaunchAngle(double angle) {
        this.angle = Range.clip(angle, 20, 45);
    }

    public double getTargetDeg() { return targetTurretDeg; }
    public double getAngleDeg() { return tracker.getTotalAngleDeg() / SERVO_TO_TURRET_GEAR_RATIO; }
    public double getVelocityDps() { return tracker.getEstimatedVelocityDps() / SERVO_TO_TURRET_GEAR_RATIO; }
    public double getCommand() { return lastCmd; }
    public double getErrorDeg() { return targetTurretDeg - getAngleDeg(); }
    public double getAnalogVoltage() { return tracker.getVoltage(); }
    public double getEstimatedPos() { return tracker.getEstimatedPosition(); }
    public double getLaunchAngle() { return angle; }

    public void stop() { turret.setPower(0); }
}
