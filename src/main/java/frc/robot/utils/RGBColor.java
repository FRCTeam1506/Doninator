package frc.robot.utils;

public class RGBColor {
    public int r, g, b;
    public RGBColor (int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public static final RGBColor RED    = new RGBColor(255, 0, 0);
    public static final RGBColor YELLOW = new RGBColor(255, 255, 0);
    public static final RGBColor GREEN  = new RGBColor(0, 255, 0);
    public static final RGBColor GRAY   = new RGBColor(128, 128, 128);
    public static final RGBColor PINK   = new RGBColor(255, 20, 147);
}