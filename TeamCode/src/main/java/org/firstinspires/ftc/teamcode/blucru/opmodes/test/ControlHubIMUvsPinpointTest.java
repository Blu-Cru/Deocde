package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class ControlHubIMUvsPinpointTest extends BluLinearOpMode {
    private IMU controlHubImu;
    private GoBildaPinpointDriver pinpoint;

    public void initialize(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        controlHubImu = hardwareMap.get(IMU.class, "imu");
        controlHubImu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    public void periodic(){
        pinpoint.update();
        telemetry.addData("Pinpoint Heading", pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("CHUB Heading", Math.toDegrees(controlHubImu.getRobotYawPitchRollAngles().getYaw()));
    }
}
