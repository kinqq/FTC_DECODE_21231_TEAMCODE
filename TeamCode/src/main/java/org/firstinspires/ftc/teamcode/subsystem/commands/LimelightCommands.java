package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.constant.Constants.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class LimelightCommands {

    public enum Motif {
        GPP, PGP, PPG, UNKNOWN
    }

    private Motif lastDetectedMotif = Motif.UNKNOWN;

    private final Limelight3A limelight;

    public LimelightCommands(HardwareMap hwMap) {
        this.limelight = hwMap.get(Limelight3A.class, "limelight");
    }

    public void start(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public Motif getLastDetectedMotif() {
        return lastDetectedMotif;
    }

    public Motif motifFromId(int id) {
        switch (id) {
            case LIMELIGHT_TAG_GPP: return Motif.GPP;
            case LIMELIGHT_TAG_PGP: return Motif.PGP;
            case LIMELIGHT_TAG_PPG: return Motif.PPG;
            default: return Motif.UNKNOWN;
        }
    }

    public Motif detectMotifFromLatest() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Motif.UNKNOWN;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Motif.UNKNOWN;
        }

        int id = fiducials.get(0).getFiducialId();

        lastDetectedMotif = motifFromId(id);
        return lastDetectedMotif;
    }

    public static class MotifMeasurement {
        public final Motif motif;
        public final int tagId;
        public final double xDeg;
        public final double yDeg;

        public MotifMeasurement(Motif motif, int tagId, double xDeg, double yDeg) {
            this.motif = motif;
            this.tagId = tagId;
            this.xDeg = xDeg;
            this.yDeg = yDeg;
        }
    }

    public MotifMeasurement getMotifMeasurementFromLatest() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        LLResultTypes.FiducialResult fid = fiducials.get(0);
        int id = fid.getFiducialId();
        double x = fid.getTargetXDegrees();
        double y = fid.getTargetYDegrees();

        Motif motif = motifFromId(id);
        return new MotifMeasurement(motif, id, x, y);
    }


    public class WaitForAnyMotif extends CommandBase {
        ElapsedTime timer = new ElapsedTime();
        boolean start = false;

        @Override
        public void initialize() {
            lastDetectedMotif = Motif.UNKNOWN;
        }

        @Override
        public void execute() {
            if (!start) {
                timer.reset();
                start = true;
            }

            lastDetectedMotif = detectMotifFromLatest();
            Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
            telemetry.addData("motif", lastDetectedMotif);
            telemetry.update();
        }

        @Override
        public boolean isFinished() {
            if (timer.seconds() > LIMELIGHT_WAIT_MOTIF_TIMEOUT_SEC) {
                lastDetectedMotif = Motif.PPG;
                return true;
            }
            return lastDetectedMotif != Motif.UNKNOWN;
        }
    }

    public CommandBase waitForAnyMotif() {
        return new WaitForAnyMotif();
    }
}
