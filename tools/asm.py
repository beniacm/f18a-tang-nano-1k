#!/usr/bin/env python3
"""
Minimal F18A assembler for the Tang Nano 1K softcore.

Source syntax (one instruction word per line):
  label:                       define a label at the current word address
  op0 [op1 [op2 [op3]]]        pack up to 4 slots; unused slots become '.'
  opX:label  or  opX:NNN       branch instruction targeting label/address
                               (opX ∈ jump, call, if, -if, next)
  [value]                      emit an 18-bit literal word verbatim
                               (value may be a number, label, #define name,
                               #variable name, or `-NAME` / `name+N` etc.)
  #origin 0xNN                 set code origin (default 0). Labels resolve
                               relative to this, so code executes at
                               origin..origin+N-1.
  #define NAME VAL             named constant (decimal/0x/0o/0b/-…). Use
                               anywhere a literal address would go.
  #variable NAME               post-program scratch cell. Auto-allocated
                               at the next free word past the last code
                               cell, so it can never collide with code.
                               Multiple #variables stack in declaration
                               order. Cells are implicitly zero on load.
  # ...                        comment

Slot-3 opcodes must fit in 3 bits: ret(;), ex, unext, @p, !p, +*, 2*, 2/.
(Octal opcodes 0..7 after zero-extending — see mnemonic table below.)

Branches in slot 0 consume the remaining 13 bits as address; in slot 1 the
low 8 bits; in slot 2 the low 3 bits; slot 3 is invalid for branches.

Output:
  firmware_init.vh   — Verilog $include fragment with `mem[N] = 18'hXXXXX;`
  firmware.lst       — listing (addr: octal disasm   hex)
"""
import re
import sys

# ── F18A opcode table (octal) ───────────────────────────────
OPS = {
    ';':    0o00, 'ret':  0o00,
    'ex':   0o01,
    'jump': 0o02,
    'call': 0o03,
    'unext':0o04,
    'next': 0o05,
    'if':   0o06,
    '-if':  0o07,
    '@p':   0o10,
    '@+':   0o11,
    '@b':   0o12,
    '@':    0o13,
    '!p':   0o14,
    '!+':   0o15,
    '!b':   0o16,
    '!':    0o17,
    '+*':   0o20,
    '2*':   0o21,
    '2/':   0o22,
    '-':    0o23, 'inv': 0o23,
    '+':    0o24,
    'and':  0o25,
    'xor':  0o26, 'or': 0o26,
    'drop': 0o27,
    'dup':  0o30,
    'pop':  0o31,
    'over': 0o32,
    'a':    0o33,
    '.':    0o34, 'nop': 0o34,
    'push': 0o35,
    'b!':   0o36,
    'a!':   0o37,
}

BRANCH = {'jump', 'call', 'if', '-if', 'next'}
# Slot 3 stores 3 bits — the high 3 bits of a 5-bit op (low 2 bits = 00).
# That gives 8 valid ops in slot 3: every fourth entry in the OPS table.
# Matches real F18A's `last_slot_ops` set so ga-tools' aforth output
# drops in unmodified.
SLOT3_OK = {0o00, 0o04, 0o10, 0o14, 0o20, 0o24, 0o30, 0o34}  # ; unext @p !p +* + dup .

def assemble_to_words(src_path, size=1024):
    """Assemble src_path. Returns (origin, words, listing, symbols, vars):
    - words is a list of 18-bit ints (program code only),
    - symbols is the merged label/define/variable map,
    - vars is {name: addr} for auto-allocated post-program scratch cells."""
    with open(src_path) as f:
        raw = f.readlines()
    return _assemble_raw(src_path, raw, size)


# Recognise a directive at the start of a stripped line. The prefix '#' would
# otherwise just be a comment, so we have to match before stripping comments.
DIRECTIVE_RE = re.compile(
    r'^#(origin|define|variable)\b\s*(.*?)\s*$'
)


def _scan_directives(raw):
    """First pass: pull out #origin / #define / #variable.  Returns
    (origin, defines, variable_names_in_order, kept_lines)
    where kept_lines is the list of (lineno, raw_text) for non-directive
    lines, used by the next pass to assign code addresses."""
    origin = 0
    defines = {}
    variables = []
    seen = set()
    kept = []
    for lineno, line in enumerate(raw, 1):
        s = line.strip()
        m = DIRECTIVE_RE.match(s)
        if not m:
            kept.append((lineno, line))
            continue
        kind, rest = m.group(1), m.group(2)
        # Strip a trailing `# ...` comment from the directive value.
        # (The directive itself starts with `#`, so we can't pre-strip.)
        rest = rest.split('#', 1)[0].strip()
        if kind == 'origin':
            if not rest:
                die(f"line {lineno}: #origin needs a value")
            origin = parse_num(rest, {})
        elif kind == 'define':
            parts = rest.split(None, 1)
            if len(parts) != 2:
                die(f"line {lineno}: #define NAME VALUE")
            name, val = parts
            if not re.match(r'^[A-Za-z_]\w*$', name):
                die(f"line {lineno}: bad #define name {name!r}")
            if name in seen:
                die(f"line {lineno}: duplicate symbol {name!r}")
            defines[name] = parse_num(val, defines)
            seen.add(name)
        elif kind == 'variable':
            name = rest.strip()
            if not re.match(r'^[A-Za-z_]\w*$', name):
                die(f"line {lineno}: bad #variable name {name!r}")
            if name in seen:
                die(f"line {lineno}: duplicate symbol {name!r}")
            variables.append(name)
            seen.add(name)
    return origin, defines, variables, kept


def _assemble_raw(src_path, raw, size):
    origin, defines, variables, kept = _scan_directives(raw)

    # ── Pass 1: walk code lines, collect labels, count words ──
    labels = {}
    tokens_per_line = []
    addr = origin
    for lineno, line in kept:
        line = line.split('#', 1)[0].strip()
        if not line:
            tokens_per_line.append(None); continue
        m = re.match(r'^([A-Za-z_][\w-]*):$', line)
        if m:
            name = m.group(1)
            if name in labels or name in defines or name in variables:
                die(f"{src_path}:{lineno}: duplicate symbol {name!r}")
            labels[name] = addr
            tokens_per_line.append(None); continue
        tokens_per_line.append((lineno, line))
        addr += 1
    nwords = addr - origin

    # ── Allocate variables right past the last code word ──
    var_addrs = {}
    for i, name in enumerate(variables):
        var_addrs[name] = origin + nwords + i
    end = origin + nwords + len(variables)
    if end - origin > size:
        die(f"program + variables exceed {size} words ({end - origin})")

    # Merged symbol table for parse_num — labels for jump targets, defines
    # for arbitrary constants, variables for scratch addresses.
    symbols = {}
    symbols.update(defines)
    symbols.update(labels)
    symbols.update(var_addrs)

    # ── Pass 2: emit words ──
    words = []
    listing = []
    idx = 0
    for entry in tokens_per_line:
        if entry is None:
            continue
        lineno, line = entry
        try:
            if line.startswith('[') and line.endswith(']'):
                val = parse_num(line[1:-1].strip(), symbols)
                w = val & 0x3FFFF
            else:
                w = encode_word(line, origin + idx, symbols)
            words.append(w)
            listing.append((origin + idx, line, w))
            idx += 1
        except AsmError as e:
            die(f"{src_path}:{lineno}: {e}: {line!r}")
    return origin, words, listing, symbols, var_addrs


def assemble(src_path, vh_path, lst_path, size=1024):
    with open(src_path) as f:
        raw = f.readlines()

    origin, prog_words, listing, symbols, var_addrs = _assemble_raw(src_path, raw, size)
    nwords = len(prog_words)
    words = [0] * size
    for i, w in enumerate(prog_words):
        words[origin + i] = w

    # ── Emit Verilog ROM module ──
    # Combinational case — async read. The core's ST_FETCH/ST_FWAIT and
    # ST_MEMRD expect memory to respond within one cycle *through* the
    # mem_addr register, i.e. no additional latency beyond that register,
    # i.e. async-read memory. Port width is fixed at 6 bits to cover the
    # full 64-word ROM region (addresses 0x40..0x7F in the SoC); extra
    # upper bits of the index are ignored.
    rom_bits = 6
    rom_path = vh_path
    with open(rom_path, 'w') as f:
        f.write(f"// auto-generated by asm.py — do not edit (origin=0x{origin:03X}, {nwords} words)\n")
        f.write("module firmware_rom(\n")
        f.write(f"    input  wire  [{rom_bits-1}:0] addr,\n")
        f.write("    output reg  [17:0] data\n")
        f.write(");\n")
        f.write("    always @(*) begin\n")
        f.write("        case (addr)\n")
        for i, w in enumerate(words[:nwords]):
            f.write(f"            {rom_bits}'h{i:X}: data = 18'h{w:05X};\n")
        f.write("            default: data = 18'h00000;\n")
        f.write("        endcase\n")
        f.write("    end\n")
        f.write("endmodule\n")

    # ── Emit listing ──
    with open(lst_path, 'w') as f:
        f.write(f"; {src_path} → {vh_path}\n")
        f.write(f"; origin=0x{origin:03X}  {nwords} words used of {size}\n")
        _write_var_listing(f, var_addrs)
        f.write("\n")
        for a, src, w in listing:
            f.write(f"{a:03X}: {w:05X}   {src}\n")

    print(f"assembled {nwords} words @ 0x{origin:03X} → {rom_path}")


def _write_var_listing(f, var_addrs):
    """Emit a `; var NAME = 0xAAA` line for each #variable, in address order."""
    for name, addr in sorted(var_addrs.items(), key=lambda x: x[1]):
        f.write(f"; var {name} = 0x{addr:03X}\n")

class AsmError(Exception):
    pass

def die(msg):
    print("asm error:", msg, file=sys.stderr)
    sys.exit(1)

def parse_num(s, labels):
    s = s.strip()
    if s in labels:
        return labels[s]
    # Unary minus on a symbol — useful for `[-VAR]` so the program can
    # add a negated address without hand-encoding the two's complement.
    if s.startswith('-') and s[1:] in labels:
        return -labels[s[1:]]
    if s.startswith('0x') or s.startswith('0X'):
        return int(s, 16)
    if s.startswith('0o'):
        return int(s, 8)
    if s.startswith('0b'):
        return int(s, 2)
    return int(s, 0)

def encode_word(line, addr, labels):
    toks = line.split()
    slots = []
    branch_idx = None
    branch_tgt = None

    for i, t in enumerate(toks):
        if ':' in t and not t.startswith('0') and t not in ('b!', 'a!'):
            base, tgt = t.split(':', 1)
            if base not in BRANCH:
                raise AsmError(f"bad branch op {base!r}")
            if branch_idx is not None:
                raise AsmError("multiple branches in one word")
            branch_idx = i
            branch_tgt = tgt
            slots.append(OPS[base])
            # branches consume remainder of word as address — stop here
            break
        else:
            if t not in OPS:
                raise AsmError(f"unknown mnemonic {t!r}")
            slots.append(OPS[t])

    if len(slots) > 4:
        raise AsmError("more than 4 slots")

    # Pad missing slots: slots 1-2 default to `.` (nop), slot 3 defaults to
    # `;` (ret) which the core treats as "fetch next word" at rsp==0.
    while len(slots) < 3:
        slots.append(OPS['.'])
    if len(slots) < 4:
        slots.append(OPS[';'])

    # Validate slot-3 restriction (only if no branch pushing slot-3 into address field)
    if branch_idx is None:
        if slots[3] not in SLOT3_OK:
            raise AsmError(
                f"opcode {slots[3]:o} (octal) cannot go in slot 3; "
                f"valid ops are ; unext @p !p +* + dup ."
            )

    # Pack. Slot 3 stores op>>2 (the high 3 bits of a 5-bit op); the
    # core re-extends them with two zero LSBs at decode time.
    w = (slots[0] << 13) | (slots[1] << 8) | (slots[2] << 3) | ((slots[3] >> 2) & 0b111)

    # If there's a branch, overwrite the tail of the word with the address.
    if branch_idx is not None:
        tgt = parse_num(branch_tgt, labels)
        if branch_idx == 0:
            # bits [12:0]
            w = (slots[0] << 13) | (tgt & 0x1FFF)
        elif branch_idx == 1:
            # bits [7:0]
            w = (slots[0] << 13) | (slots[1] << 8) | (tgt & 0xFF)
        elif branch_idx == 2:
            # bits [2:0]
            w = (slots[0] << 13) | (slots[1] << 8) | (slots[2] << 3) | (tgt & 0x7)
        else:
            raise AsmError("branch cannot be in slot 3")

    return w & 0x3FFFF


def assemble_to_hex(src_path, hex_path, lst_path, mem_size=1024):
    """Like `assemble`, but emits a flat $readmemh-compatible hex file
    (one 5-hex-digit word per line, mem_size lines total). Words before
    the program origin and after the last word are zero. Intended for
    standalone simulations where the entire address space is one RAM."""
    origin, words, listing, symbols, var_addrs = assemble_to_words(src_path, size=mem_size)
    flat = [0] * mem_size
    for i, w in enumerate(words):
        flat[origin + i] = w

    with open(hex_path, 'w') as f:
        f.write(f"// {src_path} → {hex_path}  (origin=0x{origin:03X}, "
                f"{len(words)} words used of {mem_size})\n")
        for w in flat:
            f.write(f"{w:05x}\n")

    with open(lst_path, 'w') as f:
        f.write(f"; {src_path} → {hex_path}\n")
        f.write(f"; origin=0x{origin:03X}  {len(words)} words used of {mem_size}\n")
        _write_var_listing(f, var_addrs)
        f.write("\n")
        for a, src, w in listing:
            f.write(f"{a:03X}: {w:05X}   {src}\n")

    print(f"assembled {len(words)} words @ 0x{origin:03X} → {hex_path}")


if __name__ == "__main__":
    args = sys.argv[1:]
    # Hex mode: -hex foo.f18a foo.hex foo.lst [mem_size]
    if args and args[0] == "-hex":
        if len(args) not in (4, 5):
            print("usage: asm.py -hex src.f18a out.hex out.lst [mem_size]", file=sys.stderr)
            sys.exit(2)
        size = int(args[4]) if len(args) == 5 else 1024
        assemble_to_hex(args[1], args[2], args[3], mem_size=size)
        sys.exit(0)
    if len(args) not in (1, 3):
        print("usage: asm.py src.f18a [firmware_init.vh firmware.lst]\n"
              "       asm.py -hex src.f18a out.hex out.lst [mem_size]", file=sys.stderr)
        sys.exit(2)
    src = args[0]
    vh  = args[1] if len(args) > 1 else "firmware_init.vh"
    lst = args[2] if len(args) > 2 else "firmware.lst"
    assemble(src, vh, lst)
