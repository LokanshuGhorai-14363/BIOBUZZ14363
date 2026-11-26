package util;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * MultipleTelemetry — Forwards FTC Telemetry calls to multiple Telemetry receivers
 * (e.g. Driver Station Telemetry + Panels Web Dashboard Telemetry).
 */
public class MultipleTelemetry implements Telemetry {

    private final List<Telemetry> telemetries = new ArrayList<>();

    public MultipleTelemetry(Telemetry... telemetries) {
        if (telemetries != null) {
            for (Telemetry t : telemetries) {
                if (t != null) {
                    this.telemetries.add(t);
                }
            }
        }
    }

    @Override
    public Item addData(String caption, Object value) {
        for (Telemetry t : telemetries) {
            t.addData(caption, value);
        }
        return null;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        for (Telemetry t : telemetries) {
            t.addData(caption, format, args);
        }
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        for (Telemetry t : telemetries) {
            t.addData(caption, valueProducer);
        }
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        for (Telemetry t : telemetries) {
            t.addData(caption, format, valueProducer);
        }
        return null;
    }

    @Override
    public Line addLine() {
        for (Telemetry t : telemetries) {
            t.addLine();
        }
        return null;
    }

    @Override
    public Line addLine(String lineCaption) {
        for (Telemetry t : telemetries) {
            t.addLine(lineCaption);
        }
        return null;
    }

    @Override
    public boolean removeItem(Item item) {
        boolean removed = false;
        for (Telemetry t : telemetries) {
            if (t.removeItem(item)) {
                removed = true;
            }
        }
        return removed;
    }

    @Override
    public boolean removeLine(Line line) {
        boolean removed = false;
        for (Telemetry t : telemetries) {
            if (t.removeLine(line)) {
                removed = true;
            }
        }
        return removed;
    }

    @Override
    public void clear() {
        for (Telemetry t : telemetries) {
            t.clear();
        }
    }

    @Override
    public void clearAll() {
        for (Telemetry t : telemetries) {
            t.clearAll();
        }
    }

    @Override
    public boolean update() {
        boolean updated = false;
        for (Telemetry t : telemetries) {
            if (t.update()) {
                updated = true;
            }
        }
        return updated;
    }

    @Override
    public Object addAction(Runnable action) {
        Object token = null;
        for (Telemetry t : telemetries) {
            token = t.addAction(action);
        }
        return token;
    }

    @Override
    public boolean removeAction(Object token) {
        boolean removed = false;
        for (Telemetry t : telemetries) {
            if (t.removeAction(token)) {
                removed = true;
            }
        }
        return removed;
    }

    @Override
    public void speak(String text) {
        for (Telemetry t : telemetries) {
            t.speak(text);
        }
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        for (Telemetry t : telemetries) {
            t.speak(text, languageCode, countryCode);
        }
    }

    @Override
    public boolean isAutoClear() {
        if (!telemetries.isEmpty()) {
            return telemetries.get(0).isAutoClear();
        }
        return true;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        for (Telemetry t : telemetries) {
            t.setAutoClear(autoClear);
        }
    }

    @Override
    public int getMsTransmissionInterval() {
        if (!telemetries.isEmpty()) {
            return telemetries.get(0).getMsTransmissionInterval();
        }
        return 20;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        for (Telemetry t : telemetries) {
            t.setMsTransmissionInterval(msTransmissionInterval);
        }
    }

    @Override
    public String getItemSeparator() {
        if (!telemetries.isEmpty()) {
            return telemetries.get(0).getItemSeparator();
        }
        return ", ";
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        for (Telemetry t : telemetries) {
            t.setItemSeparator(itemSeparator);
        }
    }

    @Override
    public String getCaptionValueSeparator() {
        if (!telemetries.isEmpty()) {
            return telemetries.get(0).getCaptionValueSeparator();
        }
        return ": ";
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        for (Telemetry t : telemetries) {
            t.setCaptionValueSeparator(captionValueSeparator);
        }
    }



    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        for (Telemetry t : telemetries) {
            t.setDisplayFormat(displayFormat);
        }
    }

    @Override
    public Log log() {
        if (!telemetries.isEmpty()) {
            return telemetries.get(0).log();
        }
        return null;
    }
}
