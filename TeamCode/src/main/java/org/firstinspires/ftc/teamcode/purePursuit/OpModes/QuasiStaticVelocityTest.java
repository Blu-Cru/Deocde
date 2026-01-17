package org.firstinspires.ftc.teamcode.purePursuit.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.purePursuit.Math.AsymmetricProfile.DirectTrajectory;
import org.firstinspires.ftc.teamcode.purePursuit.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.purePursuit.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands.DriveTrajectory;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands.QuasiStaticVelocity;

import java.util.ArrayList;

@Autonomous
public class QuasiStaticVelocityTest extends BaseAuto {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        robot.odometry.setEstimate(new Vector(new double[]{0, 0, 0}));

        QuasiStaticVelocity quasiStaticTest = new QuasiStaticVelocity(robot.drivetrain, robot.odometry, 0.25, 30);

        return quasiStaticTest;
    }
}
