namespace PrismTestApp;

/// <summary>
/// Four distinct LED patterns: two static, two animated.
/// Each pattern fills a byte[] of length pixelCount*3 (RGB triplets).
/// </summary>
public static class Patterns
{
    /// <summary>
    /// Animated rainbow that cycles through the strip over time.
    /// </summary>
    public static void Rainbow(byte[] buffer, int pixelCount, double timeSeconds)
    {
        double offset = timeSeconds * 60.0; // pixels/sec scroll speed
        for (int i = 0; i < pixelCount; i++)
        {
            double hue = ((i + offset) % pixelCount) / pixelCount * 360.0;
            HsvToRgb(hue, 1.0, 1.0, out byte r, out byte g, out byte b);
            buffer[i * 3] = r;
            buffer[i * 3 + 1] = g;
            buffer[i * 3 + 2] = b;
        }
    }

    /// <summary>
    /// Animated "larson scanner" (bouncing dot) — a bright white pixel sweeps back and forth
    /// with a fading red trail.
    /// </summary>
    public static void LarsonScanner(byte[] buffer, int pixelCount, double timeSeconds)
    {
        // Clear to black
        Array.Clear(buffer, 0, buffer.Length);

        const double speed = 40.0; // pixels per second
        double pos = timeSeconds * speed;
        // Triangle wave for bouncing
        double period = (pixelCount - 1) * 2;
        if (period < 1) period = 1;
        double phase = pos % period;
        double ledPos = phase <= pixelCount - 1 ? phase : period - phase;

        // Draw trail (6 pixels) + bright head
        const int trailLength = 6;
        for (int i = 0; i < pixelCount; i++)
        {
            double dist = Math.Abs(i - ledPos);
            if (dist < 1.0)
            {
                // Bright white head
                buffer[i * 3] = 255;
                buffer[i * 3 + 1] = 255;
                buffer[i * 3 + 2] = 255;
            }
            else if (dist < trailLength)
            {
                // Red fading trail
                double brightness = 1.0 - (dist / trailLength);
                byte val = (byte)(brightness * brightness * 200);
                buffer[i * 3] = val;
                buffer[i * 3 + 1] = 0;
                buffer[i * 3 + 2] = 0;
            }
        }
    }

    /// <summary>
    /// Static solid blue gradient — bright at center, fading to edges.
    /// </summary>
    public static void BlueBreath(byte[] buffer, int pixelCount, double timeSeconds)
    {
        _ = timeSeconds; // static pattern, time ignored
        double center = (pixelCount - 1) / 2.0;
        double halfLen = pixelCount / 2.0;
        if (halfLen < 1) halfLen = 1;

        for (int i = 0; i < pixelCount; i++)
        {
            double dist = Math.Abs(i - center) / halfLen;
            double brightness = 1.0 - dist;
            if (brightness < 0) brightness = 0;
            byte b = (byte)(brightness * 255);
            byte g = (byte)(brightness * 40); // slight cyan tint
            buffer[i * 3] = 0;
            buffer[i * 3 + 1] = g;
            buffer[i * 3 + 2] = b;
        }
    }

    /// <summary>
    /// Static alternating green/gold stripes (4-pixel wide).
    /// </summary>
    public static void GreenGoldStripes(byte[] buffer, int pixelCount, double timeSeconds)
    {
        _ = timeSeconds; // static pattern
        const int stripeWidth = 4;

        for (int i = 0; i < pixelCount; i++)
        {
            bool isGreen = ((i / stripeWidth) % 2) == 0;
            if (isGreen)
            {
                buffer[i * 3] = 0;
                buffer[i * 3 + 1] = 200;
                buffer[i * 3 + 2] = 0;
            }
            else
            {
                buffer[i * 3] = 255;
                buffer[i * 3 + 1] = 180;
                buffer[i * 3 + 2] = 0;
            }
        }
    }

    // ========================= Helpers ======================================

    private static void HsvToRgb(double h, double s, double v, out byte r, out byte g, out byte b)
    {
        h %= 360;
        if (h < 0) h += 360;
        double c = v * s;
        double x = c * (1.0 - Math.Abs((h / 60.0) % 2.0 - 1.0));
        double m = v - c;

        double r1, g1, b1;
        if (h < 60) { r1 = c; g1 = x; b1 = 0; }
        else if (h < 120) { r1 = x; g1 = c; b1 = 0; }
        else if (h < 180) { r1 = 0; g1 = c; b1 = x; }
        else if (h < 240) { r1 = 0; g1 = x; b1 = c; }
        else if (h < 300) { r1 = x; g1 = 0; b1 = c; }
        else { r1 = c; g1 = 0; b1 = x; }

        r = (byte)((r1 + m) * 255);
        g = (byte)((g1 + m) * 255);
        b = (byte)((b1 + m) * 255);
    }
}
