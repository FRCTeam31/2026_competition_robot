using System.Diagnostics;
using System.IO.Ports;
using PrismTestApp;

// ========================= Configuration ====================================

const int pixelsPerStrip = 8;           // Adjust to match your physical strips
const int targetFps = 60;
const double rotationIntervalSec = 5.0;
const int baudRate = 2_000_000;

// ========================= Pattern Registry =================================

// Delegate: fills buffer[0..pixelCount*3-1] with RGB data given the current time
delegate void PatternFunc(byte[] buffer, int pixelCount, double timeSec);

PatternFunc[] patterns =
[
    Patterns.Rainbow,          // animated
    Patterns.LarsonScanner,    // animated
    Patterns.BlueBreath,       // static
    Patterns.GreenGoldStripes, // static
];

string[] patternNames =
[
    "Rainbow (animated)",
    "Larson Scanner (animated)",
    "Blue Gradient (static)",
    "Green/Gold Stripes (static)",
];

// ========================= COM Port Selection ===============================

Console.WriteLine("=== Prism LED Controller Test App ===");
Console.WriteLine();

var ports = SerialPort.GetPortNames();
if (ports.Length == 0)
{
    Console.ForegroundColor = ConsoleColor.Red;
    Console.WriteLine("No serial ports found. Connect the Prism device and try again.");
    Console.ResetColor();
    return 1;
}

Console.WriteLine("Available serial ports:");
for (int i = 0; i < ports.Length; i++)
    Console.WriteLine($"  [{i}] {ports[i]}");

Console.Write($"Select port [0-{ports.Length - 1}]: ");
string? input = Console.ReadLine();
if (!int.TryParse(input, out int portIndex) || portIndex < 0 || portIndex >= ports.Length)
{
    Console.WriteLine("Invalid selection.");
    return 1;
}

string portName = ports[portIndex];

// ========================= Connect ==========================================

Console.WriteLine($"Opening {portName} at {baudRate} baud...");

using var serial = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
{
    ReadTimeout = 200,
    WriteTimeout = 200,
    DtrEnable = true,
    RtsEnable = true,
};

try
{
    serial.Open();
}
catch (Exception ex)
{
    Console.ForegroundColor = ConsoleColor.Red;
    Console.WriteLine($"Failed to open port: {ex.Message}");
    Console.ResetColor();
    return 1;
}

Console.ForegroundColor = ConsoleColor.Green;
Console.WriteLine("Connected!");
Console.ResetColor();

// ========================= Heartbeat ========================================

Console.Write("Sending heartbeat... ");
SendFrame(serial, PrismProtocol.BuildHeartbeatRequest());
Thread.Sleep(100);

if (TryReadResponse(serial, out var cmd, out var status, out var uptimeMs, out var fwVersion) && cmd == PrismProtocol.CmdHeartbeatRsp)
{
    Console.ForegroundColor = ConsoleColor.Green;
    Console.WriteLine($"OK — uptime {uptimeMs}ms, firmware v{fwVersion:X4}, status {status}");
    Console.ResetColor();
}
else
{
    Console.ForegroundColor = ConsoleColor.Yellow;
    Console.WriteLine("No heartbeat response (device may still work).");
    Console.ResetColor();
}

// ========================= Configure Strips =================================

Console.WriteLine($"Configuring {PrismProtocol.StripCount} strips × {pixelsPerStrip} pixels...");

for (byte strip = 0; strip < PrismProtocol.StripCount; strip++)
{
    var configFrame = PrismProtocol.BuildConfigureFrame(strip, (ushort)pixelsPerStrip, PrismProtocol.ColorOrderGrb);
    SendFrame(serial, configFrame);
    Thread.Sleep(50);

    if (TryReadResponse(serial, out cmd, out status, out _, out _) && cmd == PrismProtocol.CmdConfigAck)
    {
        string statusText = status == PrismProtocol.StatusOk ? "OK" : "ERROR";
        Console.WriteLine($"  Strip {strip}: {statusText}");
    }
    else
    {
        Console.ForegroundColor = ConsoleColor.Yellow;
        Console.WriteLine($"  Strip {strip}: no ACK");
        Console.ResetColor();
    }
}

// ========================= Allocate Buffers =================================

var stripBuffers = new byte[PrismProtocol.StripCount][];
for (int i = 0; i < PrismProtocol.StripCount; i++)
    stripBuffers[i] = new byte[pixelsPerStrip * 3];

// ========================= Main Loop ========================================

Console.WriteLine();
Console.WriteLine("Streaming patterns. Press Ctrl+C or Q to quit.");
Console.WriteLine();

int rotationOffset = 0;
var sw = Stopwatch.StartNew();
double lastRotationTime = 0;
int frameCount = 0;
double lastFpsTime = 0;
int lastFpsFrame = 0;
TimeSpan frameInterval = TimeSpan.FromSeconds(1.0 / targetFps);

using var cts = new CancellationTokenSource();
Console.CancelKeyPress += (_, e) => { e.Cancel = true; cts.Cancel(); };

try
{
    while (!cts.IsCancellationRequested)
    {
        double now = sw.Elapsed.TotalSeconds;

        // Rotate patterns every N seconds
        if (now - lastRotationTime >= rotationIntervalSec)
        {
            lastRotationTime = now;
            rotationOffset = (rotationOffset + 1) % patterns.Length;
            PrintStripAssignment(rotationOffset, patternNames);
        }

        // Render each strip's pattern
        for (int strip = 0; strip < PrismProtocol.StripCount; strip++)
        {
            int patternIdx = (strip + rotationOffset) % patterns.Length;
            patterns[patternIdx](stripBuffers[strip], pixelsPerStrip, now);
        }

        // Send all pixel data in one frame
        var frame = PrismProtocol.BuildPixelDataAllFrame(stripBuffers);
        SendFrame(serial, frame);

        frameCount++;

        // Print FPS every 2 seconds
        if (now - lastFpsTime >= 2.0)
        {
            double elapsed = now - lastFpsTime;
            double fps = (frameCount - lastFpsFrame) / elapsed;
            Console.Write($"\r  {fps:F1} fps | frame {frameCount} | rotation {rotationOffset}    ");
            lastFpsFrame = frameCount;
            lastFpsTime = now;
        }

        // Check for Q key (non-blocking)
        if (Console.KeyAvailable)
        {
            var key = Console.ReadKey(true);
            if (key.Key == ConsoleKey.Q)
                break;
        }

        // Frame rate limiting
        var next = TimeSpan.FromTicks((long)(frameCount * frameInterval.Ticks));
        var sleepTime = next - sw.Elapsed;
        if (sleepTime > TimeSpan.Zero)
            Thread.Sleep(sleepTime);
    }
}
catch (OperationCanceledException)
{
    // Expected on Ctrl+C
}
catch (Exception ex)
{
    Console.ForegroundColor = ConsoleColor.Red;
    Console.WriteLine($"\nError: {ex.Message}");
    Console.ResetColor();
}

// ========================= Cleanup =========================================

Console.WriteLine("\n\nShutting down — blanking strips...");

// Send all-black frame
for (int i = 0; i < PrismProtocol.StripCount; i++)
    Array.Clear(stripBuffers[i], 0, stripBuffers[i].Length);

SendFrame(serial, PrismProtocol.BuildPixelDataAllFrame(stripBuffers));
Thread.Sleep(50);

serial.Close();
Console.WriteLine("Done.");
return 0;

// ========================= Helpers ==========================================

static void SendFrame(SerialPort port, byte[] frame)
{
    try
    {
        port.Write(frame, 0, frame.Length);
    }
    catch (TimeoutException)
    {
        // Non-fatal: device might be busy
    }
}

static bool TryReadResponse(SerialPort port, out byte command, out byte status, out uint uptimeMs, out ushort firmwareVersion)
{
    command = 0;
    status = 0;
    uptimeMs = 0;
    firmwareVersion = 0;

    try
    {
        int available = port.BytesToRead;
        if (available < PrismProtocol.FrameOverhead)
        {
            Thread.Sleep(50);
            available = port.BytesToRead;
        }

        if (available <= 0)
            return false;

        var buf = new byte[Math.Min(available, 64)];
        int read = port.Read(buf, 0, buf.Length);
        return PrismProtocol.TryParseResponse(buf.AsSpan(0, read), out command, out status, out uptimeMs, out firmwareVersion) > 0;
    }
    catch (TimeoutException)
    {
        return false;
    }
}

static void PrintStripAssignment(int offset, string[] names)
{
    Console.WriteLine();
    for (int strip = 0; strip < PrismProtocol.StripCount; strip++)
    {
        int idx = (strip + offset) % names.Length;
        Console.ForegroundColor = strip switch
        {
            0 => ConsoleColor.Cyan,
            1 => ConsoleColor.Red,
            2 => ConsoleColor.Blue,
            3 => ConsoleColor.Yellow,
            _ => ConsoleColor.White,
        };
        Console.WriteLine($"  Strip {strip}: {names[idx]}");
    }
    Console.ResetColor();
}
