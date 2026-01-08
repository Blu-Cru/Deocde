package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.*;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ResetForIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class CommandFactory {

    public static Command createCommand(String commandName, JSONArray args, JSONArray children) throws JSONException {
        switch (commandName) {
            case "SequentialCommandGroup":
                return createSequentialCommandGroup(children);
            case "ParallelCommandGroup": // Support parallel if needed
                // return createParallelCommandGroup(children); // Implement similar to
                // Sequential
                break;
            case "WaitCommand":
                return new WaitCommand(args.getLong(0));
            case "IntakeCommand":
                return new ResetForIntakeCommand(); // Check constructor args if any
            case "TransferCommand":
                return new TransferCommand(args.getBoolean(0));
            case "AutonomousShootCloseCommand":
                return new AutonomousShootCloseCommand();
            case "AutonomousShootCommand":
                return new AutonomousShootCommand(); // Verify constructor
            case "ShootWithVelocityCommand":
                return new ShootWithVelocityCommand(args.getDouble(0));
            case "IntakeStartCommand":
                return new IntakeStartCommand();
            case "ElevatorDownCommand":
                return new ElevatorDownCommand();
            case "AutonomousTransferCommand":
                // 850, 26, 28, 26
                return new AutonomousTransferCommand(
                        args.getDouble(0),
                        args.getDouble(1),
                        args.getDouble(2),
                        args.getDouble(3));
            // Add other commands here
            default:
                throw new IllegalArgumentException("Unknown command: " + commandName);
        }
        return null; // Should not reach here
    }

    private static Command createSequentialCommandGroup(JSONArray children) throws JSONException {
        SequentialCommandGroup group = new SequentialCommandGroup();
        if (children != null) {
            for (int i = 0; i < children.length(); i++) {
                JSONObject childObj = children.getJSONObject(i);
                String cmdName = childObj.getString("command");
                JSONArray cmdArgs = childObj.optJSONArray("args");
                JSONArray cmdChildren = childObj.optJSONArray("children");
                group.addCommands(createCommand(cmdName, cmdArgs, cmdChildren));
            }
        }
        return group;
    }
}
