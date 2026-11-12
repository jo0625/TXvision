"""Independent periodic memory endpoint for the actual Windows duplex helper.

The records are synthetic counters, not robot/IMU messages. This factory never
constructs ComEndpoint or opens a physical serial port.
"""

from pathlib import Path


def load_helper_script(_port, _baud, *, trace=False, fault="none", delay_ms=0):
    if fault not in {"none", "slow_write", "write_timeout", "write_error", "read_error"}:
        raise ValueError(f"Unknown fault: {fault}")
    if not 0 <= delay_ms <= 1000:
        raise ValueError("delay_ms must be between 0 and 1000")
    source = (Path(__file__).resolve().parents[1] /
              "scripts/windows_duplex_helper.cs").read_text()
    endpoint = r'''
public sealed class PeriodicMemoryEndpoint : IComEndpoint {
    private readonly ConcurrentQueue<byte[]> records = new ConcurrentQueue<byte[]>();
    private readonly ManualResetEventSlim stop = new ManualResetEventSlim(false);
    private readonly Task producer;
    private readonly string fault = "__FAULT__";
    private readonly int delayMs = __DELAY__;
    private int attempted = 0, completed = 0, generated = 0;
    private long writeStart = 0, writeEnd = 0;
    private static long Us() {
        return (long)(Stopwatch.GetTimestamp() * (1000000.0 / Stopwatch.Frequency));
    }
    private void ResourceSample(string phase) {
        using (var process = Process.GetCurrentProcess()) {
            Console.Error.WriteLine("MEMORY_PROBE phase={0} pid={1} generated={2} attempted={3} completed={4} queued={5} handles={6} threads={7} working_set={8}",
                phase, process.Id, generated, attempted, completed, records.Count,
                process.HandleCount, process.Threads.Count, process.WorkingSet64);
        }
    }
    public PeriodicMemoryEndpoint() {
        ResourceSample("start");
        producer = Task.Run(() => {
            long next = Us();
            while (!stop.IsSet) {
                long now = Us();
                if (now < next) { stop.Wait(1); continue; }
                var record = new byte[38];
                record[0] = (byte)'M'; record[1] = (byte)'B';
                Array.Copy(BitConverter.GetBytes((uint)Interlocked.Increment(ref generated)), 0, record, 2, 4);
                Array.Copy(BitConverter.GetBytes(now), 0, record, 6, 8);
                Array.Copy(BitConverter.GetBytes((uint)Volatile.Read(ref attempted)), 0, record, 14, 4);
                Array.Copy(BitConverter.GetBytes((uint)Volatile.Read(ref completed)), 0, record, 18, 4);
                Array.Copy(BitConverter.GetBytes(Interlocked.Read(ref writeStart)), 0, record, 22, 8);
                Array.Copy(BitConverter.GetBytes(Interlocked.Read(ref writeEnd)), 0, record, 30, 8);
                records.Enqueue(record);
                if (records.Count > 1000) throw new IOException("Memory probe queue exceeded limit");
                if (generated % 200 == 0) ResourceSample("sample");
                next += 50000;
            }
        });
    }
    public int Available {
        get {
            if (producer.IsFaulted) producer.GetAwaiter().GetResult();
            return records.Count * 38;
        }
    }
    public int Read(byte[] buffer) {
        if (fault == "read_error" && Volatile.Read(ref attempted) >= 5)
            throw new IOException("Injected memory endpoint read error");
        int count = 0;
        byte[] record;
        while (count + 38 <= buffer.Length && records.TryDequeue(out record)) {
            Array.Copy(record, 0, buffer, count, 38);
            count += 38;
        }
        return count;
    }
    public void Write(byte[] data, int timeoutMs) {
        if (data.Length != 11) throw new IOException("Expected opaque 11 byte probe record");
        int attempt = Interlocked.Increment(ref attempted);
        Interlocked.Exchange(ref writeStart, Us());
        if (attempt == 5) {
            Console.Error.WriteLine("MEMORY_PROBE write5 fault={0} timeout_ms={1} delay_ms={2}", fault, timeoutMs, delayMs);
            if (fault == "slow_write") Thread.Sleep(delayMs);
            if (fault == "write_timeout") {
                Thread.Sleep(timeoutMs);
                throw new TimeoutException("Injected memory endpoint write timeout");
            }
            if (fault == "write_error") throw new IOException("Injected memory endpoint write error");
        }
        Interlocked.Increment(ref completed);
        Interlocked.Exchange(ref writeEnd, Us());
    }
    public void Dispose() {
        stop.Set();
        try { producer.GetAwaiter().GetResult(); }
        finally { ResourceSample("stop"); stop.Dispose(); }
    }
}
'''
    endpoint = endpoint.replace("__FAULT__", fault).replace("__DELAY__", str(delay_ms))
    # Keep the actual helper plus fixture below Windows' encoded command-line
    # limit. Only indentation, blank lines and full-line comments are removed.
    source = "\n".join(line.strip() for line in (source + endpoint).splitlines()
                       if line.strip() and not line.lstrip().startswith("//"))
    return (
        '$ErrorActionPreference = "Stop"\n'
        "Add-Type -TypeDefinition @'\n" + source + "\n'@\n"
        '[DuplexPipe]::Run([PeriodicMemoryEndpoint]::new()'
        + (", $true)\n" if trace else ")\n")
    )
