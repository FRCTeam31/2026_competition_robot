namespace PrismTestApp;

/// <summary>
/// Prism binary serial protocol — matches the Java/C implementations exactly.
/// Frame: [0xAA][0x55][cmd:1][payloadLen:2 LE][payload...][XOR checksum:1]
/// </summary>
public static class PrismProtocol
{
    // Sync bytes
    public const byte SyncByte1 = 0xAA;
    public const byte SyncByte2 = 0x55;

    // Commands (Host -> Device)
    public const byte CmdConfigure = 0x01;
    public const byte CmdPixelData = 0x02;
    public const byte CmdPixelDataAll = 0x03;
    public const byte CmdHeartbeatReq = 0x04;

    // Commands (Device -> Host)
    public const byte CmdConfigAck = 0x81;
    public const byte CmdHeartbeatRsp = 0x84;

    // Status codes
    public const byte StatusOk = 0x00;
    public const byte StatusError = 0x01;

    // Limits
    public const int StripCount = 4;
    public const int MaxPixelsPerStrip = 144;
    public const int FrameOverhead = 6; // 2 sync + 1 cmd + 2 len + 1 checksum

    // Color orders
    public const byte ColorOrderRgb = 0x00;
    public const byte ColorOrderGrb = 0x01;

    /// <summary>
    /// Compute XOR checksum over a payload span.
    /// </summary>
    public static byte Checksum(ReadOnlySpan<byte> payload)
    {
        byte xor = 0;
        foreach (var b in payload)
            xor ^= b;
        return xor;
    }

    /// <summary>
    /// Wrap a payload into a complete frame: [sync1][sync2][cmd][len LE][payload][checksum].
    /// </summary>
    public static byte[] WrapFrame(byte command, ReadOnlySpan<byte> payload)
    {
        var frame = new byte[FrameOverhead + payload.Length];
        frame[0] = SyncByte1;
        frame[1] = SyncByte2;
        frame[2] = command;
        frame[3] = (byte)(payload.Length & 0xFF);
        frame[4] = (byte)((payload.Length >> 8) & 0xFF);
        payload.CopyTo(frame.AsSpan(5));
        frame[^1] = Checksum(payload);
        return frame;
    }

    /// <summary>
    /// Build a CONFIGURE frame for a single strip.
    /// Payload: [strip:1][pixelCount:2 LE][colorOrder:1]
    /// </summary>
    public static byte[] BuildConfigureFrame(byte strip, ushort pixelCount, byte colorOrder = ColorOrderGrb)
    {
        Span<byte> payload = stackalloc byte[4];
        payload[0] = strip;
        payload[1] = (byte)(pixelCount & 0xFF);
        payload[2] = (byte)((pixelCount >> 8) & 0xFF);
        payload[3] = colorOrder;
        return WrapFrame(CmdConfigure, payload);
    }

    /// <summary>
    /// Build a PIXEL_DATA_ALL frame containing pixel data for all 4 strips.
    /// Payload: for each strip → [pixelCount:2 LE][R,G,B × pixelCount]
    /// </summary>
    public static byte[] BuildPixelDataAllFrame(byte[][] stripBuffers)
    {
        int payloadSize = 0;
        foreach (var buf in stripBuffers)
            payloadSize += 2 + buf.Length;

        var payload = new byte[payloadSize];
        int offset = 0;

        foreach (var buf in stripBuffers)
        {
            int pixelCount = buf.Length / 3;
            payload[offset++] = (byte)(pixelCount & 0xFF);
            payload[offset++] = (byte)((pixelCount >> 8) & 0xFF);
            Buffer.BlockCopy(buf, 0, payload, offset, buf.Length);
            offset += buf.Length;
        }

        return WrapFrame(CmdPixelDataAll, payload);
    }

    /// <summary>
    /// Build a HEARTBEAT_REQ frame (empty payload).
    /// </summary>
    public static byte[] BuildHeartbeatRequest()
    {
        return WrapFrame(CmdHeartbeatReq, ReadOnlySpan<byte>.Empty);
    }

    /// <summary>
    /// Try to parse a CONFIG_ACK or HEARTBEAT_RSP response from a byte buffer.
    /// Returns the number of bytes consumed, or 0 if no complete frame found.
    /// </summary>
    public static int TryParseResponse(ReadOnlySpan<byte> data, out byte command, out byte status, out uint uptimeMs, out ushort firmwareVersion)
    {
        command = 0;
        status = 0;
        uptimeMs = 0;
        firmwareVersion = 0;

        // Scan for sync bytes
        for (int i = 0; i <= data.Length - FrameOverhead; i++)
        {
            if (data[i] != SyncByte1 || data[i + 1] != SyncByte2)
                continue;

            byte cmd = data[i + 2];
            ushort payloadLen = (ushort)(data[i + 3] | (data[i + 4] << 8));
            int frameLen = FrameOverhead + payloadLen;

            if (i + frameLen > data.Length)
                return 0; // incomplete

            var payload = data.Slice(i + 5, payloadLen);
            byte expectedChecksum = Checksum(payload);
            if (data[i + frameLen - 1] != expectedChecksum)
            {
                continue; // bad checksum, skip this sync pair
            }

            command = cmd;

            if (cmd == CmdConfigAck && payloadLen >= 2)
            {
                // payload: [strip][status]
                status = payload[1];
            }
            else if (cmd == CmdHeartbeatRsp && payloadLen >= 7)
            {
                // payload: [uptimeMs:4 LE][firmwareVersion:2 LE][status:1]
                uptimeMs = (uint)(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));
                firmwareVersion = (ushort)(payload[4] | (payload[5] << 8));
                status = payload[6];
            }

            return i + frameLen;
        }

        return 0;
    }
}
