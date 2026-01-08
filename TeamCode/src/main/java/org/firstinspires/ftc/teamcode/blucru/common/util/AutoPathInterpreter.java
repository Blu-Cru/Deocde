package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.HashMap;

public class AutoPathInterpreter {
    JSONObject root;

    public AutoPathInterpreter(String fileName){
        try {
            root = readJSONFile(fileName);
        } catch (Exception e) {
            throw new RuntimeException("Error reading JSON: " + e.getMessage(), e);
        }
    }

    public Pose2d getStartPose() {
        try {
            if (root.has("startPose")) {
                JSONObject startPoseObj = root.getJSONObject("startPose");
                double x = startPoseObj.getDouble("x");
                double y = startPoseObj.getDouble("y");
                double heading = Math.toRadians(startPoseObj.getDouble("headingDegrees"));
                return new Pose2d(x, y, heading);
            }
            return new Pose2d(0,0,0);
        } catch (Exception e) {
            throw new RuntimeException("Error reading startPose from JSON: " + e.getMessage(), e);
        }
    }

    public double[] getStartingHoodAngles(){
        try {
            if (root.has("startingHoodAngles")) {

                JSONObject hoodAnglesObj = root.getJSONObject("startingHoodAngles");
                return new double[] {
                        hoodAnglesObj.getDouble("leftHoodAngle"),
                        hoodAnglesObj.getDouble("middleHoodAngle"),
                        hoodAnglesObj.getDouble("rightHoodAngle")
                };
            }
        } catch (Exception e){
            throw new RuntimeException("cannot get starting info");
        }

        return new double[]{26,26,26};

    }

    public double getStartingShooterVel(){
        try{
            if (root.has("startingShooterVel")) {

                JSONObject shooterVelObj = root.getJSONObject("startingShooterVel");
                return shooterVelObj.getDouble("vel");
            }
        } catch (Exception e) {
            throw new RuntimeException("cannot get starting shooter velocity");
        }
        return 0;
    }

    public double getStartingTurretAngle(){
        try{
            if (root.has("startingTurretAngle")) {

                JSONObject shooterVelObj = root.getJSONObject("startingTurretAngle");
                return shooterVelObj.getDouble("angle");
            }
        } catch (Exception e) {
            throw new RuntimeException("cannot get starting shooter velocity");
        }
        return 0;
    }

    public Action buildPathFromJSON(Pose2d startPose, TankDrive drive) {
        Globals.telemetry.addData("StartingPose", startPose);
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        try {
            JSONArray actions = root.getJSONArray("steps");

            for (int i = 0; i < actions.length(); i++) {
                JSONObject step = actions.getJSONObject(i);
                String type = step.getString("type");

                switch (type) {
                    case "splineTo":
                        double x = step.getDouble("x");
                        double y = step.getDouble("y");
                        double tangent = Math.toRadians(step.getDouble("tangentDegrees"));
                        if (step.has("velConstraint")) {
                            double vel = step.getDouble("velConstraint");
                            builder = builder.splineTo(new Vector2d(x, y), tangent,
                                    new TranslationalVelConstraint(vel));
                        } else {
                            builder = builder.splineTo(new Vector2d(x, y), tangent);
                        }
                        break;
                    case "lineToX":
                        builder = builder.lineToX(step.getDouble("x"));
                        break;
                    case "lineToY":
                        builder = builder.lineToY(step.getDouble("y"));
                        break;
                    case "turnTo":
                        builder = builder.turnTo(Math.toRadians(step.getDouble("headingDegrees")));
                        break;
                    case "waitSeconds":
                        builder = builder.waitSeconds(step.getDouble("seconds"));
                        break;
                    case "setReversed":
                        builder = builder.setReversed(step.getBoolean("reversed"));
                        break;
                    case "stopAndAdd":
                        String cmdName = step.getString("command");
                        JSONArray args = step.optJSONArray("args");
                        JSONArray children = step.optJSONArray("children");
                        Command command = CommandFactory.createCommand(cmdName, args, children);
                        builder = builder.stopAndAdd(new FtclibCommandAction(command));
                        break;
                    case "afterTime":
                        double delay = step.getDouble("delay");
                        String afterCmdName = step.getString("command");
                        JSONArray afterArgs = step.optJSONArray("args");
                        JSONArray afterChildren = step.optJSONArray("children");
                        Command afterCommand = CommandFactory.createCommand(afterCmdName, afterArgs, afterChildren);
                        builder = builder.afterTime(delay, new FtclibCommandAction(afterCommand));
                        break;
                }
            }
        } catch (Exception e) {
            throw new RuntimeException("Error parsing path JSON: " + e.getMessage(), e);
        }

        return builder.build();
    }

    public String generateJavaCodeFromJSON(String filePath) {
        StringBuilder code = new StringBuilder();
        try {
            JSONObject root = readJSONFile(filePath);

            // Start Pose
            if (root.has("startPose")) {
                JSONObject sp = root.getJSONObject("startPose");
                code.append(String.format("Pose2d startPose = new Pose2d(%.2f, %.2f, Math.toRadians(%.2f));\n",
                        sp.getDouble("x"), sp.getDouble("y"), sp.getDouble("headingDegrees")));
            } else {
                code.append("// No startPose in JSON\n");
            }

            code.append("Action path = drive.actionBuilder(startPose)\n");

            JSONArray actions = root.getJSONArray("steps");
            for (int i = 0; i < actions.length(); i++) {
                JSONObject step = actions.getJSONObject(i);
                String type = step.getString("type");

                switch (type) {
                    case "splineTo":
                        code.append(String.format("    .splineTo(new Vector2d(%.2f, %.2f), Math.toRadians(%.2f)",
                                step.getDouble("x"), step.getDouble("y"), step.getDouble("tangentDegrees")));
                        if (step.has("velConstraint")) {
                            code.append(String.format(", new TranslationalVelConstraint(%.2f)",
                                    step.getDouble("velConstraint")));
                        }
                        code.append(")\n");
                        break;
                    case "lineToX":
                        code.append(String.format("    .lineToX(%.2f)\n", step.getDouble("x")));
                        break;
                    case "lineToY":
                        code.append(String.format("    .lineToY(%.2f)\n", step.getDouble("y")));
                        break;
                    case "turnTo":
                        code.append(
                                String.format("    .turnTo(Math.toRadians(%.2f))\n", step.getDouble("headingDegrees")));
                        break;
                    case "waitSeconds":
                        code.append(String.format("    .waitSeconds(%.2f)\n", step.getDouble("seconds")));
                        break;
                    case "setReversed":
                        code.append(String.format("    .setReversed(%b)\n", step.getBoolean("reversed")));
                        break;
                    case "stopAndAdd":
                        code.append(String.format("    .stopAndAdd(new FtclibCommandAction(%s))\n",
                                formatCommandConstructor(step.getString("command"), step.optJSONArray("args"),
                                        step.optJSONArray("children"))));
                        break;
                    case "afterTime":
                        code.append(String.format("    .afterTime(%.2f, new FtclibCommandAction(%s))\n",
                                step.getDouble("delay"),
                                formatCommandConstructor(step.getString("command"), step.optJSONArray("args"),
                                        step.optJSONArray("children"))));
                        break;
                }
            }
            code.append("    .build();\n");

        } catch (Exception e) {
            return "Error generating code: " + e.getMessage();
        }
        return code.toString();
    }

    private static String formatCommandConstructor(String name, JSONArray args, JSONArray children)
            throws org.json.JSONException {
        StringBuilder sb = new StringBuilder();
        sb.append("new ").append(name).append("(");

        if (name.equals("SequentialCommandGroup") && children != null) {
            for (int i = 0; i < children.length(); i++) {
                if (i > 0)
                    sb.append(", ");
                JSONObject child = children.getJSONObject(i);
                sb.append(formatCommandConstructor(child.getString("command"), child.optJSONArray("args"),
                        child.optJSONArray("children")));
            }
        } else if (args != null) {
            for (int i = 0; i < args.length(); i++) {
                if (i > 0)
                    sb.append(", ");
                Object val = args.get(i);
                sb.append(val.toString());
            }
        }

        sb.append(")");
        return sb.toString();
    }

    private JSONObject readJSONFile(String filePath) throws java.io.IOException, org.json.JSONException {
        File file = new File(filePath);
        BufferedReader reader = new BufferedReader(new FileReader(file));
        StringBuilder jsonString = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            jsonString.append(line);
        }
        reader.close();
        return new JSONObject(jsonString.toString());
    }
}
