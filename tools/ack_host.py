#!/usr/bin/env python3
"""Run the Ackermann benchmark on the Tang Nano 1K F18A SoC.

The bitstream comes up at the streaming INST_PORT (0x3F9). The host
streams a tiny F18A loader followed by the program payload, then a
`jump:0` to leave port-execution and run from RAM. After that the
host streams the three benchmark inputs (count, n_init, m_init) as
plain 18-bit words — the program reads them via `@b` reads with
B = INST_PORT, so there are no magic input addresses, just three
port mailbox reads.

Loader scheme — one `@p !+` per cell:

    @p a! NOP RET         — A ← target load address (next streamed word)
    [target=0]
                          — then for each cell:
    @p !+ NOP RET         — store next streamed word at mem[A], A++
    [data_word]
                          — repeat (one instr + one data word per cell)
    jump:0                — leave port-execution, run from RAM[0]

The simpler one-cell-per-instruction approach avoids UNEXT's
loop-back-to-current-word semantic, which doesn't work from a port:
P stays at the port across the fetch, so `current_word = P - 1` lands
on RAM, not back at the port. Costs ~2× the stream bytes vs an inner
loop, still ≪ a millisecond per cell at 115200 baud.

After the loader runs, the program at mem[0] sets B = INST_PORT and
does three `@b` reads to pull count/n_init/m_init from the host stream.
The `mem_ready` handshake makes those reads block until the host has
sent enough bytes for one assembled word, so the host can keep on
writing without worrying about the program racing ahead.

Re-flash (`make flash-fullisa-sram`) between runs to get a clean POR;
the FPGA holds T/S/R/A/I across reset per the F18A spec, but P comes
back to INST_PORT so the loader will stream into a fresh memory.
"""
import os, re, sys, time, argparse, serial

TTY  = os.environ.get("TTY", "/dev/ttyUSB0")
BAUD = int(os.environ.get("BAUD", "115200"))


def load_hex(path):
    """Return a list of 18-bit words read from asm.py -hex output (// header
    indicates how many cells are meaningful)."""
    raw = []
    used = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("//"):
                m = re.search(r"(\d+)\s+words used of\s+(\d+)", line)
                if m:
                    used = int(m.group(1))
                continue
            raw.append(int(line, 16))
    if used is None:
        used = len(raw)
        while used > 0 and raw[used - 1] == 0:
            used -= 1
    return raw[:used]


def word_bytes(w):
    """Pack an 18-bit instruction word into 3 UART bytes (lo, mid, hi[1:0])."""
    return bytes([w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0x03])


# F18A instruction words used by the streamed loader.
BOOT_LOAD_A    = 0x11FE0   # @p a! NOP RET   — A ← next streamed word
BOOT_STORE_INC = 0x10DE0   # @p !+ NOP RET   — mem[A] ← next; A++
BOOT_JUMP_0    = 0x04000   # jump:0          — slot-0 jump, 13-bit addr


def stream_load_and_run(s, words, *, target=0):
    """Stream a loader + words[] + jump:target.  Returns total bytes sent."""
    if not words:
        return 0
    payload = [BOOT_LOAD_A, target]
    for w in words:
        payload += [BOOT_STORE_INC, w]
    payload.append(BOOT_JUMP_0 | (target & 0x1FFF))
    blob = b"".join(word_bytes(w) for w in payload)
    s.write(blob); s.flush()
    return len(blob)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", type=int, default=3, help="m_init  (default 3)")
    ap.add_argument("-n", type=int, default=2, help="n_init  (default 2)")
    ap.add_argument("-r", "--repeat", type=int, default=1,
                    help="repeat the Ackermann computation this many times inside the core")
    ap.add_argument("--hex", default="/tmp/ack.hex",
                    help="hex file emitted by asm.py -hex (default /tmp/ack.hex)")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="seconds to wait for the final result byte")
    args = ap.parse_args()

    if args.repeat < 1:
        print("error: --repeat must be >= 1", file=sys.stderr)
        sys.exit(2)
    if args.m > 3 or args.n > 9:
        print(f"warning: A({args.m},{args.n}) may overflow the 8-bit UART byte",
              file=sys.stderr)

    prog = load_hex(args.hex)
    print(f"loaded {len(prog)} words from {args.hex}")

    s = serial.Serial(TTY, BAUD, timeout=0.3)
    time.sleep(0.05)
    s.reset_input_buffer()

    t0 = time.monotonic()
    print(f"streaming {len(prog)}-cell program + bootloader to INST_PORT")
    nbytes = stream_load_and_run(s, prog, target=0)
    print(f"  sent {nbytes} bytes ({nbytes // 3} instruction words)")

    # Stream the three input words. Program reads them via @b at startup
    # (B = INST_PORT). The blocking-port handshake makes order safe.
    print(f"streaming inputs: count={args.repeat} n={args.n} m={args.m}")
    s.write(word_bytes(args.repeat))
    s.write(word_bytes(args.n))
    s.write(word_bytes(args.m))
    s.flush()
    print(f"running A({args.m}, {args.n}) x {args.repeat} ...")

    # Wait for the program's single result byte. Anything else (FTDI
    # idle 0xFF, possibly a stray byte left over from a previous run)
    # is filtered.
    deadline = t0 + args.timeout
    got = bytearray()
    while time.monotonic() < deadline:
        r = s.read(64)
        if r:
            got += bytes(c for c in r if c != 0xFF)
            if len(got) >= 1:
                break

    elapsed_ms = 1000 * (time.monotonic() - t0)
    s.close()

    if not got:
        print(f"FAIL — no result byte after {args.timeout:.1f}s")
        sys.exit(1)

    result = got[0]
    per_iter_ms = elapsed_ms / args.repeat
    iter_per_sec = 1000.0 / per_iter_ms if per_iter_ms > 0 else float("inf")
    if args.repeat == 1:
        print(f"A({args.m}, {args.n}) = {result}   ({elapsed_ms:.1f} ms)")
    else:
        print(f"A({args.m}, {args.n}) x {args.repeat} = {result}   "
              f"({elapsed_ms:.1f} ms total, {per_iter_ms:.3f} ms/iter, "
              f"{iter_per_sec:.1f} iter/s)")

    expected_lowbyte = {
        (0, 0): 1, (1, 1): 3, (2, 2): 7, (2, 3): 9,
        (3, 1): 13, (3, 2): 29, (3, 3): 61, (3, 4): 125, (3, 5): 253,
    }
    e = expected_lowbyte.get((args.m, args.n))
    if e is not None and result != e:
        print(f"FAIL — expected {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
