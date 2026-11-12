using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;

public interface IComEndpoint : IDisposable {
    int Available { get; }
    int Read(byte[] buffer);
    void Write(byte[] buffer, int timeoutMs);
}

public sealed class ComEndpoint : IComEndpoint {
    private readonly SerialPort port;
    public ComEndpoint(string name, int baud) {
        port = new SerialPort(name, baud, Parity.None, 8, StopBits.One);
        port.Handshake = Handshake.None;
        port.DtrEnable = false;
        port.RtsEnable = false;
        port.ReadTimeout = 20;
        try { port.Open(); }
        catch { port.Dispose(); throw; }
    }
    public int Available { get { return port.BytesToRead; } }
    public int Read(byte[] buffer) { return port.Read(buffer, 0, buffer.Length); }
    public void Write(byte[] buffer, int timeoutMs) {
        port.WriteTimeout = timeoutMs;
        port.Write(buffer, 0, buffer.Length);
    }
    public void Dispose() { port.Dispose(); }
}

public static class DuplexPipe {
    private static long MonotonicMs() {
        return (long)(Stopwatch.GetTimestamp() * (1000.0 / Stopwatch.Frequency));
    }

    private static long MonotonicUs() {
        return (long)(Stopwatch.GetTimestamp() * (1000000.0 / Stopwatch.Frequency));
    }

    private sealed class PipeOutput : IDisposable {
        private readonly BinaryWriter output = new BinaryWriter(Console.OpenStandardOutput());
        private readonly object gate = new object();
        private bool closed;

        public void Ready() {
            output.Write(System.Text.Encoding.ASCII.GetBytes("READY\n"));
            output.Flush();
        }

        public void Frame(byte kind, byte[] data, int count) {
            lock (gate) {
                if (closed) return;
                // ACK, clock and RX writers must never interleave a pipe frame.
                output.Write(kind);
                output.Write((uint)count);
                output.Write(data, 0, count);
                output.Flush();
            }
        }

        public void Ack(uint sequence, byte status) {
            var ack = new byte[5];
            Array.Copy(BitConverter.GetBytes(sequence), ack, 4);
            ack[4] = status;
            Frame(2, ack, ack.Length);
        }

        public void Dispose() {
            lock (gate) {
                if (closed) return;
                closed = true;
                output.Dispose();
            }
        }
    }

    private sealed class WriteRequest {
        public uint Sequence;
        public long Expires;
        public byte[] Data;
    }

    private sealed class RunState {
        public volatile bool Stopping;
    }

    private static void ReadRequests(BlockingCollection<WriteRequest> requests,
                                     PipeOutput output, RunState state) {
        bool syncReady = false;
        try {
            string line;
            while (!state.Stopping && (line = Console.ReadLine()) != null && line != "stop") {
                if (state.Stopping) break;
                if (line.Length > 1600) throw new IOException("Command line too long");
                var parts = line.Split(' ');
                if (parts.Length == 2 && parts[0] == "SYNC") {
                    var sync = new byte[12];
                    Array.Copy(BitConverter.GetBytes(UInt32.Parse(parts[1])), 0, sync, 0, 4);
                    Array.Copy(BitConverter.GetBytes(MonotonicMs()), 0, sync, 4, 8);
                    output.Frame(3, sync, sync.Length);
                    syncReady = true;
                    continue;
                }
                if (parts.Length != 3) throw new IOException("Invalid command envelope");
                if (!syncReady) throw new IOException("Missing Windows clock handshake");
                var request = new WriteRequest {
                    Sequence = UInt32.Parse(parts[0]),
                    Expires = Int64.Parse(parts[1]),
                    Data = Convert.FromBase64String(parts[2])
                };
                if (request.Data.Length == 0 || request.Data.Length > 1024)
                    throw new IOException("Invalid command length");
                while (!state.Stopping && !requests.TryAdd(request)) {
                    WriteRequest superseded;
                    if (requests.TryTake(out superseded))
                        output.Ack(superseded.Sequence, 3); // Replaced without executing.
                }
            }
        } finally {
            state.Stopping = true;
            requests.CompleteAdding();
        }
    }

    private static void WriteRequests(IComEndpoint port, BlockingCollection<WriteRequest> requests,
                                      PipeOutput output, RunState state) {
        while (!state.Stopping) {
            WriteRequest request;
            if (!requests.TryTake(out request, 10)) continue;
            if (state.Stopping) break; // Never drain queued commands on shutdown.
            long remaining = request.Expires - MonotonicMs();
            byte status = remaining > 1000 ? (byte)2 : (byte)1;
            if (remaining > 0 && remaining <= 1000) {
                try {
                    port.Write(request.Data, (int)remaining);
                } catch (Exception error) {
                    // A write may have partially completed: fail the connection,
                    // report its stage, and never retry or acknowledge success.
                    throw new IOException("Windows serial write failed (request " +
                                          request.Sequence + "): " + error.Message, error);
                }
                status = 0;
            }
            output.Ack(request.Sequence, status);
        }
    }

    public static void Run(IComEndpoint port) {
        Run(port, false);
    }

    public static void Run(IComEndpoint port, bool trace) {
        using (port)
        using (var output = new PipeOutput()) {
            var state = new RunState();
            var requests = new BlockingCollection<WriteRequest>(1);
            output.Ready();
            var input = Task.Factory.StartNew(() => ReadRequests(requests, output, state),
                CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);
            var writer = Task.Factory.StartNew(() => WriteRequests(port, requests, output, state),
                CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);
            var buffer = new byte[4096];
            long lastClock = MonotonicMs();
            try {
                while (!state.Stopping) {
                    if (writer.IsFaulted) writer.GetAwaiter().GetResult();
                    long readStart = trace ? MonotonicUs() : 0;
                    int count = 0;
                    try {
                        if (port.Available > 0) count = port.Read(buffer);
                    } catch (Exception error) {
                        throw new IOException("Windows serial read failed: " + error.Message, error);
                    }
                    if (count > 0 && trace) {
                        long readEnd = MonotonicUs();
                        var timed = new byte[16 + count];
                        Array.Copy(BitConverter.GetBytes(readStart), 0, timed, 0, 8);
                        Array.Copy(BitConverter.GetBytes(readEnd), 0, timed, 8, 8);
                        Array.Copy(buffer, 0, timed, 16, count);
                        output.Frame(5, timed, timed.Length);
                    } else if (count > 0) {
                        output.Frame(1, buffer, count);
                    }
                    long now = MonotonicMs();
                    if (now - lastClock >= 20) {
                        var clock = BitConverter.GetBytes(now);
                        output.Frame(4, clock, clock.Length);
                        lastClock = now;
                    }
                    Thread.Sleep(1);
                }
                input.GetAwaiter().GetResult();
            } finally {
                state.Stopping = true;
                // The endpoint must outlive an active Write. Its maximum valid
                // deadline is 1 s; allow margin, then let the parent kill a hung
                // helper instead of reporting a successful shutdown.
                bool stopped = true;
                try { stopped = writer.Wait(1500); }
                catch (AggregateException) { /* Propagate below or preserve the read error. */ }
                if (!stopped) {
                    output.Dispose();
                    throw new IOException("Windows serial write did not stop during shutdown");
                }
            }
            writer.GetAwaiter().GetResult();
        }
    }
}
