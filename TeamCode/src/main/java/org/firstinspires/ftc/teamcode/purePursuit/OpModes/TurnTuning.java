package org.firstinspires.ftc.teamcode.purePursuit.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.CommandScheduler;


@Autonomous
public class TurnTuning extends BaseAuto {
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		Command auto = turn(Math.toRadians(180));
		return auto;
	}
}
