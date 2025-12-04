package org.firstinspires.ftc.teamcode.meet2testing;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.meet2testing.subsys.*;
import org.firstinspires.ftc.teamcode.meet2testing.subsys.cmds.*;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "meet2")
public class drive extends OpMode {
    GamepadEx driverOp;
    GamepadEx toolOp;

    Button exampleButton;

    test Test;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        Test = new test(hardwareMap, "test");

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        exampleButton = new GamepadButton(driverOp, GamepadKeys.Button.A);
        runtime.reset();
    }

    @Override
    public void loop()
    {
        if (gamepad1.aWasPressed()) {

            new testcmd(Test, 0.5, runtime).schedule();
        }
        if (gamepad1.bWasPressed()) {

            new testcmd(Test, 0.1, runtime).schedule();
        }
        telemetry.addData("Runtime", runtime.seconds());
    }

}
