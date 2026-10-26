using System;
using System.Net;
using System.Net.Sockets;

namespace TizenAmbilight
{
    /// <summary>
    /// WLED realtime UDP output, DRGB protocol (WLED "UDP realtime" port 21324).
    ///
    /// Packet layout:
    ///     byte 0        0x02          protocol id: DRGB (R,G,B per LED, no separate white channel)
    ///     byte 1        timeout       seconds WLED keeps the strip live after our last packet
    ///     byte 2..      3 bytes/LED   R,G,B for LED 0, R,G,B for LED 1, ...
    ///
    /// 73 LEDs => 2 + 73*3 = 221 bytes. Sent as a single UDP datagram.
    ///
    /// SAFETY: this class never sends anything unless the app asks it to, which it does only when
    /// `outputEnabled` is true AND a `wledIp` is configured. A fresh install sends nothing.
    ///
    /// BLANKING: <see cref="SendBlackout"/> is the deliberate "turn the strip off" packet — the
    /// same DRGB frame with every LED at 0,0,0. While the app wants the strip dark it re-sends this
    /// inside the timeout, because a single blackout expires and WLED reverts to its own colour. The app sends it when capture has failed for a
    /// sustained run (the strip would otherwise sit latched on a stale frame) and from its
    /// terminate callback. A hard kill cannot run that callback, which is why every packet carries
    /// the short timeout rather than relying on the callback alone.
    /// </summary>
    public static class WledOutput
    {
        public const byte ProtocolDrgb = 0x02;
        public const int DefaultPort = 21324;
        public const byte DefaultTimeoutSeconds = 1;

        /// <summary>WLED accepts at most 490 LEDs in one DRGB datagram (2 + 490*3 = 1472 bytes, one
        /// Ethernet MTU of UDP payload). Larger strips would need the DNRGB protocol.</summary>
        public const int MaxDrgbLeds = 490;

        // ---- DDP (Distributed Display Protocol, WLED UDP port 4048) ----
        // DRGB is one datagram, so it tops out at 490 LEDs. DDP splits a frame over several datagrams
        // and marks the LAST one with a PUSH flag; WLED holds the partial frame until the push arrives,
        // so a long strip updates atomically instead of tearing between packets. Layout follows the
        // 3waylabs DDP spec (the same 10-byte header the Android sibling project wled-tv sends).
        public const int DdpPort = 4048;
        public const int DdpHeaderLen = 10;
        /// <summary>Payload bytes per DDP datagram: 480 RGB LEDs, comfortably inside one MTU.</summary>
        public const int DdpMaxPayload = 1440;
        /// <summary>Upper bound for DDP strips. Not a protocol limit; a sanity cap that the WLED
        /// controller itself (ESP8266 especially) runs out of RAM or frame time well before.</summary>
        public const int MaxDdpLeds = 1500;

        public const string ProtocolNameDrgb = "drgb";
        public const string ProtocolNameDdp = "ddp";

        public static bool IsDdp(string protocol)
        {
            return string.Equals(protocol, ProtocolNameDdp, StringComparison.OrdinalIgnoreCase);
        }

        /// <summary>LED ceiling for a protocol: one datagram for DRGB, the DDP sanity cap otherwise.</summary>
        public static int MaxLeds(string protocol)
        {
            return IsDdp(protocol) ? MaxDdpLeds : MaxDrgbLeds;
        }

        /// <summary>Default UDP port for a protocol.</summary>
        public static int DefaultPortFor(string protocol)
        {
            return IsDdp(protocol) ? DdpPort : DefaultPort;
        }

        static int _ddpSeq;     // 1..15, advanced once per frame; render thread plus shutdown, under _udpLock

        /// <summary>Wrap a DRGB payload (the output of <see cref="BuildDrgbMapped"/> or
        /// <see cref="BuildBlackout"/>) as DDP datagrams. The two header bytes of the DRGB payload are
        /// dropped; DDP carries no timeout byte, so WLED's own "realtime timeout" governs how long the
        /// strip stays live after the last frame.
        ///
        /// Every datagram of a frame shares one sequence number (1..15). Only the last carries the PUSH
        /// flag. The offset field is in BYTES (channels), not LEDs.</summary>
        public static byte[][] BuildDdp(byte[] drgb, byte sequence)
        {
            if (drgb == null) throw new ArgumentNullException(nameof(drgb));
            int total = drgb.Length - 2;
            if (total < 0) total = 0;
            int count = total == 0 ? 1 : (total + DdpMaxPayload - 1) / DdpMaxPayload;
            var packets = new byte[count][];
            for (int i = 0; i < count; i++)
            {
                int offset = i * DdpMaxPayload;
                int len = Math.Min(DdpMaxPayload, total - offset);
                if (len < 0) len = 0;
                byte[] d = new byte[DdpHeaderLen + len];
                d[0] = (byte)(0x40 | (i == count - 1 ? 0x01 : 0x00));   // version 1, PUSH on the last
                d[1] = (byte)(sequence & 0x0F);
                d[2] = 0x01;                                            // data type: RGB
                d[3] = 0x01;                                            // destination id: default device
                d[4] = (byte)(offset >> 24);
                d[5] = (byte)(offset >> 16);
                d[6] = (byte)(offset >> 8);
                d[7] = (byte)offset;
                d[8] = (byte)(len >> 8);
                d[9] = (byte)len;
                if (len > 0) Buffer.BlockCopy(drgb, 2 + offset, d, DdpHeaderLen, len);
                packets[i] = d;
            }
            return packets;
        }

        /// <summary>Send one finished frame with the chosen protocol. DRGB frames go out as the single
        /// datagram they already are; DDP frames are split by <see cref="BuildDdp"/>. Returns null on
        /// success or a short error string; never throws.</summary>
        public static string SendFrame(string ip, int port, string protocol, byte[] drgbPayload)
        {
            if (!IsDdp(protocol)) return Send(ip, port, drgbPayload);

            byte seq;
            lock (_udpLock)
            {
                _ddpSeq = _ddpSeq % 15 + 1;
                seq = (byte)_ddpSeq;
            }
            byte[][] packets = BuildDdp(drgbPayload, seq);
            for (int i = 0; i < packets.Length; i++)
            {
                string err = Send(ip, port, packets[i]);
                if (err != null) return err;
            }
            return null;
        }

        /// <summary>Timeout byte for the stall BLACKOUT packet. Deliberately longer than the normal
        /// frame timeout: the app re-sends every BlackoutHoldMs (1 s), so 4 s means up to three
        /// consecutive lost packets still leave WLED latched to black. At 1 s a single dropped datagram
        /// was enough for WLED to drop live mode and flash its own default ORANGE for the whole stall.</summary>
        public const byte BlackoutTimeoutSeconds = 4;

        static byte[] _mappedScratch;   // per-frame datagram, reused; render thread only

        /// <summary>Build the DRGB datagram for a run of zones, in strip order.</summary>
        public static byte[] BuildDrgb(ZoneColors colors, byte timeoutSeconds)
        {
            if (colors == null) throw new ArgumentNullException("colors");
            byte[] p = new byte[2 + colors.Count * 3];
            p[0] = ProtocolDrgb;
            p[1] = timeoutSeconds;
            int o = 2;
            for (int i = 0; i < colors.Count; i++)
            {
                p[o++] = colors.R[i];
                p[o++] = colors.G[i];
                p[o++] = colors.B[i];
            }
            return p;
        }

        // ---- zone -> LED MAPPING ----
        // The zone colours are computed in FRAME order: top (L->R), right (T->B), bottom (R->L),
        // left (B->T), clockwise from the frame's top-left. Where those land on the PHYSICAL strip
        // depends entirely on which corner the installer started at and which way the strip runs,
        // so both are steerable without a rebuild:
        //     mapOffset  - rotate the ring by this many LEDs (0..count-1)
        //     mapReverse - run the ring the other way
        // With ledCount > zoneCount the extra LEDs are written BLACK, NOT spread proportionally - the
        // mapping stays strictly 1:1 (see the loop below). An earlier version of this comment claimed a
        // proportional spread; that was never what the code did. Proximity-spreading drifts ~2 LEDs by
        // the end of the ring and breaks the corner alignment, which is why it was rejected.
        public static byte[] BuildDrgbMapped(ZoneColors colors, byte timeoutSeconds,
                                             double gain, double gamma, double black,
                                             int mapOffset, bool mapReverse, int ledCount)
        {
            if (colors == null) throw new ArgumentNullException(nameof(colors));
            if (ledCount < 1) ledCount = colors.Count;
            byte[] t = Lut(gain, gamma, black);
            // REUSED BUFFER. This is the per-frame builder (~25x/second) and it allocated a 227-byte
            // array every frame for a packet sent immediately and discarded. Sized EXACTLY rather than
            // ">=": Send() transmits payload.Length bytes, so an oversized buffer would emit extra LEDs.
            // Safe because the render loop sends before the next call and retains nothing.
            int need = 2 + ledCount * 3;
            if (_mappedScratch == null || _mappedScratch.Length != need) _mappedScratch = new byte[need];
            byte[] p = _mappedScratch;
            p[0] = ProtocolDrgb;
            p[1] = timeoutSeconds;

            int zones = colors.Count;
            for (int led = 0; led < ledCount; led++)
            {
                int o = 2 + led * 3;

                // LEDs past the zone count are written EXPLICITLY BLACK rather than left alone.
                // Leaving them unwritten was the bug: WLED then keeps driving them from its own
                // segment output, which is not ours and showed up as colour on the last LEDs.
                // This also keeps the zone->LED mapping strictly 1:1. A proportional spread across
                // a larger ledCount would drift ~2 LEDs by the end of the ring and break the corners.
                if (led >= zones)
                {
                    p[o] = 0; p[o + 1] = 0; p[o + 2] = 0;
                    continue;
                }

                int i = (mapReverse ? (-led) : led) + mapOffset;
                i %= zones;
                if (i < 0) i += zones;

                // Tone-map through LUMA and scale R/G/B by that SAME factor, instead of pushing each
                // channel through the curve independently. A per-channel curve lifts a low channel
                // more than a high one, which compresses their ratio - measured as ~10% relative
                // saturation loss plus a hue shift on real zone colours. Scaling by a common factor
                // changes brightness only, so hue and saturation survive the tone map intact.
                byte r0 = colors.R[i], g0 = colors.G[i], b0 = colors.B[i];
                int luma = (299 * r0 + 587 * g0 + 114 * b0) / 1000;
                int nl = t[luma];
                if (luma <= 0)
                {
                    // Neutral: no hue to preserve. (luma 0 means the zone is black or near-black.)
                    p[o] = (byte)nl; p[o + 1] = (byte)nl; p[o + 2] = (byte)nl;
                }
                else
                {
                    double k = (double)nl / luma;
                    int rr = (int)(r0 * k + 0.5), gg = (int)(g0 * k + 0.5), bb = (int)(b0 * k + 0.5);
                    p[o]     = (byte)(rr > 255 ? 255 : rr);
                    p[o + 1] = (byte)(gg > 255 ? 255 : gg);
                    p[o + 2] = (byte)(bb > 255 ? 255 : bb);
                }
            }
            return p;
        }

        // ---- brightness shaping ----
        // A dark scene averages to dark zone colours, and no amount of WLED brightness fixes that,
        // so the curve lives here. Two knobs, applied per channel through a 256-entry LUT so
        // the hot path stays a table lookup:
        //     out = clamp( gain * 255 * (v/255)^(1/gamma) )
        // gamma > 1 lifts mid-tones (the perceptual "brighter" people actually want);
        // gain is a straight multiplier on top.
        static byte[] _lut;
        static double _lutGain = double.NaN, _lutGamma = double.NaN, _lutBlack = double.NaN;

        /// <summary>Tone map, applied per channel. TWO stages, and the order matters:
        ///
        ///   1. BLACK LEVEL + CONTRAST STRETCH. Everything at or below `black` goes to zero, and the
        ///      remainder is stretched back over the full range. This is what removes the "milky"
        ///      haze: without it nothing in the chain ever crushes black, so a gamma lift turns dark
        ///      scenes into dark grey. Crucially this gives deep blacks AND bright highlights rather
        ///      than trading one against the other.
        ///
        ///   2. GAMMA / GAIN on the stretched value, for taste.
        ///
        /// black = 16 is the classic video black level (16/255); the app's default of 12 is tuned for
        /// the PQ-decoded plane (see the `black` note in Config.cs).
        /// </summary>
        static byte[] Lut(double gain, double gamma, double black)
        {
            if (_lut != null && gain == _lutGain && gamma == _lutGamma && black == _lutBlack) return _lut;
            if (gain <= 0) gain = 1.0;
            if (gamma <= 0) gamma = 1.0;
            if (black < 0) black = 0;
            if (black > 200) black = 200;
            double span = 255.0 - black;
            var t = new byte[256];
            for (int v = 0; v < 256; v++)
            {
                double x = v - black;
                if (x <= 0) { t[v] = 0; continue; }
                double n = (x / span);                       // stretch back to 0..1
                x = gain * 255.0 * Math.Pow(n, 1.0 / gamma);
                if (x < 0) x = 0; else if (x > 255) x = 255;
                t[v] = (byte)(x + 0.5);
            }
            _lut = t; _lutGain = gain; _lutGamma = gamma; _lutBlack = black;
            return t;
        }

        /// <summary>Build the all-off DRGB datagram for <paramref name="ledCount"/> LEDs: every
        /// LED 0,0,0. This is the packet that un-latches the strip.</summary>
        public static byte[] BuildBlackout(int ledCount)
        {
            if (ledCount < 1) ledCount = 1;
            byte[] p = new byte[2 + ledCount * 3];
            p[0] = ProtocolDrgb;
            p[1] = BlackoutTimeoutSeconds;
            // bytes 2.. already zero — every LED is R=0,G=0,B=0
            return p;
        }

        /// <summary>Build and send the all-off packet for a strip of <paramref name="ledCount"/>
        /// LEDs. Returns null on success or a short error string; never throws.</summary>
        public static string SendBlackout(string ip, int port, int ledCount)
        {
            return Send(ip, port, BuildBlackout(ledCount));
        }

        /// <summary>Protocol-aware blackout (DDP strips need the multi-datagram form).</summary>
        public static string SendBlackout(string ip, int port, int ledCount, string protocol)
        {
            return SendFrame(ip, port, protocol, BuildBlackout(ledCount));
        }

        static UdpClient _udp;
        static IPAddress _udpAddr;
        static int _udpPort = -1;
        static readonly object _udpLock = new object();

        /// <summary>UDP send. Returns null on success or a short error string; never throws.
        ///
        /// The socket is created ONCE and reused. The original built a UdpClient per call - a socket
        /// create + connect + close ~25 times a second for the entire runtime, which is pure syscall and
        /// GC overhead. Reuse also gives WLED one stable source port instead of a new ephemeral port per
        /// packet, which is what a realtime (live) stream should look like.
        ///
        /// LOCKED because the render loop and the shutdown handler (framework thread) can both send.
        /// A failed send DISCARDS the socket so the next call rebuilds a clean one rather than
        /// repeatedly failing on a broken handle.</summary>
        public static string Send(string ip, int port, byte[] payload)
        {
            try
            {
                IPAddress addr;
                if (!IPAddress.TryParse(ip, out addr))
                    return "bad ip '" + ip + "'";

                lock (_udpLock)
                {
                    if (_udp == null || _udpPort != port || !Equals(_udpAddr, addr))
                    {
                        try { if (_udp != null) _udp.Close(); } catch { }
                        _udp = new UdpClient();
                        _udp.Connect(addr, port);
                        _udpAddr = addr;
                        _udpPort = port;
                    }
                    _udp.Send(payload, payload.Length);
                }
                return null;
            }
            catch (Exception e)
            {
                lock (_udpLock)
                {
                    try { if (_udp != null) _udp.Close(); } catch { }
                    _udp = null; _udpPort = -1;
                }
                return e.GetType().Name + ": " + e.Message;
            }
        }
    }
}
