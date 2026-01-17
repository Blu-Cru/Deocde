package org.firstinspires.ftc.teamcode.purePursuit.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.CommandScheduler;

@Autonomous
public class BasicAuto extends BaseAuto {
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		return   drive(40)
				.addNext(turn(Math.toRadians(180)))
				.addNext(drive(-40))
				.addNext(turn(Math.toRadians(0)))
				.addNext(drive(-80));
	}
}
