package org.firstinspires.ftc.teamcode.teleop

import com.bylazar.gamepad.PanelsGamepad
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@TeleOp(name = "DriveKt")
class DriveKt : OpMode() {
    private val odo by lazy { hardwareMap.get(GoBildaPinpointDriver::class.java, "odo") }

    private val frontLeft by lazy { hardwareMap.dcMotor["leftFront"].apply { direction = DcMotorSimple.Direction.REVERSE } }
    private val backLeft by lazy { hardwareMap.dcMotor["leftBack"].apply { direction = DcMotorSimple.Direction.REVERSE } }
    private val frontRight by lazy { hardwareMap.dcMotor["rightFront"] }
    private val backRight by lazy { hardwareMap.dcMotor["rightBack"] }

    val g1 = PanelsGamepad.firstManager
    val g2 = PanelsGamepad.secondManager

    override fun init() {
        odo.apply {
            setOffsets(2.3622047244, -6.6141732283, DistanceUnit.INCH)
            setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
            )
            resetPosAndIMU()
        }

        telemetry = JoinedTelemetry(telemetry, PanelsTelemetry.ftcTelemetry)
        telemetry.addLine("Initialized!")
        telemetry.update()
    }

    override fun loop() {
        odo.update()
        val g1 = g1.asCombinedFTCGamepad(gamepad1)
        val g2 = g2.asCombinedFTCGamepad(gamepad2)

        val heading = -odo.getHeading(AngleUnit.RADIANS)

        val y = -g1.left_stick_y.toDouble()
        val x = g1.left_stick_x.toDouble() * 1.1
        val rx = g1.right_stick_x.toDouble()

        val xSpeed = x * cos(heading) - y * sin(heading)
        val ySpeed = x * sin(heading) + y * cos(heading)

        val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)

        frontLeft.power = (ySpeed + xSpeed + rx) / denominator
        backLeft.power = (ySpeed - xSpeed + rx) / denominator
        frontRight.power = (ySpeed - xSpeed - rx) / denominator
        backRight.power = (ySpeed + xSpeed - rx) / denominator

        if (g1.start) odo.resetPosAndIMU()

        telemetry.apply {
            addData("x", odo.getPosX(DistanceUnit.MM))
            addData("y", odo.getPosY(DistanceUnit.MM))
            addData("h", odo.getHeading(AngleUnit.DEGREES))
            update()
        }
    }
}
