"""Memory-only Windows endpoint for tests; never opens a COM port."""

import base64
from pathlib import Path

from test_protocol import make_feedback


def mock_helper_script(port, baud, *, fail_write=False, trace=False):
    source = (Path(__file__).resolve().parents[1] /
              'scripts/windows_duplex_helper.cs').read_text()
    mock = r'''
public sealed class MockEndpoint : IComEndpoint {
    private readonly object gate = new object();
    private byte[] pending = Convert.FromBase64String("__FEEDBACK__");
    private uint sequence = 41;
    private readonly bool failWrite = __FAIL__;
    public int Available { get { lock (gate) { return pending.Length; } } }
    public int Read(byte[] buffer) {
        lock (gate) {
            int count = pending.Length;
            Array.Copy(pending, buffer, count);
            pending = new byte[0];
            return count;
        }
    }
    public void Write(byte[] data, int timeoutMs) {
        lock (gate) {
            if (failWrite) throw new IOException("Simulated Windows serial disconnect");
            if (data.Length != 11 || data[0] != 0xA5 || data[9] != 0x5A || data[10] != 0x5B)
                throw new IOException("Unexpected command bytes");
            pending = Convert.FromBase64String("__FEEDBACK__");
            Array.Copy(BitConverter.GetBytes(++sequence), 0, pending, 1, 4);
            Array.Copy(data, 1, pending, 25, 8);
            ushort crc = 0xFFFF;
            for (int i = 0; i < 34; ++i) {
                crc ^= pending[i];
                for (int bit = 0; bit < 8; ++bit)
                    crc = (ushort)((crc >> 1) ^ ((crc & 1) != 0 ? 0x8408 : 0));
            }
            pending[34] = (byte)(crc & 0xFF);
            pending[35] = (byte)(crc >> 8);
        }
    }
    public void Dispose() {}
}
'''
    mock = mock.replace('__FEEDBACK__', base64.b64encode(make_feedback(seq=41)).decode())
    mock = mock.replace('__FAIL__', 'true' if fail_write else 'false')
    return (
        '$ErrorActionPreference = "Stop"\n'
        "Add-Type -TypeDefinition @'\n" + source + mock + "\n'@\n"
        '[DuplexPipe]::Run([MockEndpoint]::new()'
        + (', $true)\n' if trace else ')\n')
    )
