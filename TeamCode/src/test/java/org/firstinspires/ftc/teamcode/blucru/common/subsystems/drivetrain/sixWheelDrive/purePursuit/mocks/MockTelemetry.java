package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.mocks;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MockTelemetry implements Telemetry {
    @Override
    public Item addData(String caption, String format, Object... args) {
        return null;
    }

    @Override
    public Item addData(String caption, Object value) {
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> value) {
        return null;
    }

    @Override
    public Item addData(String caption, Object value, Object... args) {
        return null;
    }

    @Override
    public Line addLine() {
        return null;
    }

    @Override
    public Line addLine(String line) {
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        return false;
    }

    @Override
    public boolean removeData(Item item) {
        return false;
    }

    @Override
    public void clear() {
    }

    @Override
    public void clearAll() {
    }

    @Override
    public void update() {
    }

    @Override
    public Log log() {
        return null;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
    }

    @Override
    public int getMsTransmissionInterval() {
        return 0;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
    }

    @Override
    public String getItemSeparator() {
        return null;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
    }

    @Override
    public String getCaptionValueSeparator() {
        return null;
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
    }

    @Override
    public DisplayFormat getDisplayFormat() {
        return null;
    }

    @Override
    public boolean speak(String text) {
        return false;
    }

    @Override
    public boolean speak(String text, String languageCode, String countryCode) {
        return false;
    }
}
