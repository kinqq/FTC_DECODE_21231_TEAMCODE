package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.frontLD;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.frontLF;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.frontLI;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.frontLP;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.frontRP;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftD;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftF;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftI;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.leftP;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightD;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightF;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightI;
import static org.firstinspires.ftc.teamcode.constant.PTOPIDFConstants.rightP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp (name = "PTO Tuning", group = "Tuning")
public class PTOPIDFTuning extends OpMode
{
    DcMotorEx left;
    DcMotorEx right;

    @Override
    public void init()
    {
        left = hardwareMap.get(DcMotorEx.class, "leftFront");
        right = hardwareMap.get(DcMotorEx.class, "rightFront");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(frontLP, frontLI, frontLD, frontLF));
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(frontRP, 0, rightD, rightF));
    }

    @Override
    public void loop()
    {
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(leftP, leftI, leftD, leftF));
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(rightP, rightI, rightD, rightF));

        if (gamepad1.a)
        {
            left.setVelocity(-1500);
            right.setVelocity(-1500);
        } else
        {
            left.setPower(0);
            right.setPower(0);
        }

        telemetry.addData("Error Left", -2200 - left.getVelocity());
        telemetry.addData("Error Right", -2200 - right.getVelocity());
        telemetry.addData("Left", left.getVelocity());
        telemetry.addData("Right", right.getVelocity());
        telemetry.update();
    }
}
