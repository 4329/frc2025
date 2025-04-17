package frc.robot.subsystems.light.ledAnimations;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;

public class Movement implements LEDPattern {

    final double MULTIPLIER = 0.07;
    final double MULTIPLIER_TWO = 0.15;

    private class Point {
        int index;
        Color point;
        Color color = Color.kBlack;
        boolean hit;

        public Point(int index, Color point) {
            this.index = index;
            this.point = point;
        }
    }

    static List<Point> points = new ArrayList<>();
    static List<Integer> indices = IntStream.range(0, 130).boxed().collect(Collectors.toList());

    Color color;

    public Movement(Color color) {
        this.color = color;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        final double chance = 0.9;
        final int num = (int) Math.ceil(chance);

        for (int i = 0; i < num; i++) {
            if (Math.random() < chance / num) {
                int index = (int) (Math.random() * indices.size());
                int realdex = indices.get(index);
                indices.remove(index);
                points.add(new Point(realdex, color));
            }
        }

        for (int i = 0; i < reader.getLength(); i++) {
            writer.setLED(i, Color.kBlack);
        }

        for (int i = 0; i < points.size(); i++) {
            Point p = points.get(i);
            Color c = points.get(i).color;

            if (!p.hit) {
                points.get(i).color = Color.lerpRGB(c, p.point, MULTIPLIER_TWO);
                if (Math.abs(c.blue * c.green * c.red - p.color.blue * p.color.green * p.color.red) < 0.00001)
                    points.get(i).hit = true;
            } else {
                if (c.blue + c.green + c.red < 0.05) {
                    writer.setLED(p.index, Color.kBlack);
                    indices.add(p.index);
                    points.remove(i);
                    i--;
                    continue;
                }
                points.get(i).color = Color.lerpRGB(c, Color.kBlack, MULTIPLIER);
            }

            writer.setLED(points.get(i).index, points.get(i).color);
        }
    }
}
