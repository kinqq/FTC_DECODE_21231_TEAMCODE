package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.Selector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class SelectableOpMode extends OpMode {
    private final Selector<Supplier<OpMode>> selector;
    private OpMode selectedOpMode;
    private TelemetryManager panels;
    private final GamepadManager gm1 = PanelsGamepad.INSTANCE.getFirstManager();
    private final GamepadManager gm2 = PanelsGamepad.INSTANCE.getSecondManager();

    // 매 프레임 새로 받아올 "결합된" Gamepad (FTC 타입 그대로)
    private Gamepad g1, g2;
    private boolean prevUp, prevDown, prevA, prevB;

    private final static String[] MESSAGE = {
        "Use the d-pad to move the cursor.",
        "Press A to select.",
        "Press B to go back."
    };

    public SelectableOpMode(String name, Consumer<SelectScope<Supplier<OpMode>>> opModes) {
        selector = Selector.create(name, opModes, MESSAGE);
        selector.onSelect(opModeSupplier -> {
            onSelect();
            selectedOpMode = opModeSupplier.get();
            selectedOpMode.gamepad1 = gamepad1;
            selectedOpMode.gamepad2 = gamepad2;
            selectedOpMode.telemetry = telemetry;
            selectedOpMode.hardwareMap = hardwareMap;
            selectedOpMode.init();
        });
    }

    protected void onSelect() {
    }

    protected void onLog(List<String> line) {
    }

    @Override
    public final void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public final void init_loop() {
        g1 = gm1.asCombinedFTCGamepad(gamepad1);
        g2 = gm2.asCombinedFTCGamepad(gamepad2);

        if (selectedOpMode == null) {
            boolean curUp   = (g1.dpad_up   || g2.dpad_up);
            boolean curDown = (g1.dpad_down || g2.dpad_down);
            boolean curA    = (g1.a         || g2.a);
            boolean curB    = (g1.b         || g2.b);

            // --- edge 감지(WasPressed) ---
            boolean upWasPressed   = curUp   && !prevUp;
            boolean downWasPressed = curDown && !prevDown;
            boolean aWasPressed    = curA    && !prevA;
            boolean bWasPressed    = curB    && !prevB;

            // --- 선택 로직 ---
            if (upWasPressed)         selector.decrementSelected();
            else if (downWasPressed)  selector.incrementSelected();
            else if (aWasPressed)     selector.select();
            else if (bWasPressed)     selector.goBack();

            // --- prev 업데이트 ---
            prevUp   = curUp;
            prevDown = curDown;
            prevA    = curA;
            prevB    = curB;


//            if (gamepad1.dpad_up || gamepad2.dpad_up)
//                selector.decrementSelected();
//            else if (gamepad1.dpad_down || gamepad2.dpad_down)
//                selector.incrementSelected();
//            else if (gamepad1.a || gamepad2.a)
//                selector.select();
//            else if (gamepad1.b || gamepad2.b)
//                selector.goBack();

            List<String> lines = selector.getLines();
            for (String line : lines) {
                panels.addLine(line);
            }
            panels.update(telemetry);

            onLog(lines);
        } else selectedOpMode.init_loop();
    }

    @Override
    public final void start() {
        if (selectedOpMode == null) throw new RuntimeException("No OpMode selected!");
        selectedOpMode.start();
    }

    @Override
    public final void loop() {
        selectedOpMode.loop();
    }

    @Override
    public final void stop() {
        if (selectedOpMode != null) selectedOpMode.stop();
    }
}
