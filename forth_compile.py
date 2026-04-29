"""Forth → F18A asm compiler (host side).

Each REPL line becomes a fresh F18A program. The compiler keeps a
dictionary of `:` definitions accumulated across lines; on every
evaluatable line it produces an asm.py-compatible source for the FULLISA
user RAM (0x000..0x37F).

Memory layout (FULLISA, 1K BSRAM, user RAM 0x000..0x37F):

    0x000          jump:main          (entry trampoline)
    0x001..        runtime helpers (_emit, _swap, _0eq) — kept inside
                   page 0 so their slot-1/slot-2 branches reach
    main:          compiled top-level body for this line, ending with
                   a write to RAM[0x37E] (the "done" marker the sim
                   back-end watches for) and a halt loop
    user words:    each `:` definition compiled as a labelled sequence
                   ending in a slot-3 `;` (ret)

    0x37E          done-marker scratch (sim watches for write here)
    0x37F          _swap one-cell scratch

F18A instruction-packing notes that drove this layout:

  * Inside a `:` body (rsp >= 1), every non-terminal word ends in an
    explicit `jump:next_step` so the FSM transitions to ST_FETCH
    before slot 3, dodging the implicit slot-3 ret.
  * Slot-1 jumps reach 256 words from the current word's PC. So each
    function body — including main — must fit inside a single 256-word
    page; the assembler will produce wrong addresses if you cross.
  * Calls between sections are placed in slot 0 (full 13-bit address
    field), so user words can sit anywhere without page worries.
  * `if:_skip` for control flow goes in slot 0 alone — its address
    field consumes the rest of the word, so there's no implicit
    slot-3 ret to worry about.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Dict


# ── Tokens that compile to one F18A primitive each ────────────────────
PRIMITIVE_OPS = {
    "+", "and", "xor", "inv", "2*", "2/", "+*",
    "dup", "drop", "over",
    "push", "pop",
    "@p", "!p", "@+", "!+",
    "@", "!", "@b", "!b",
    "a", "a!", "b!",
    "ex",
}


# ── Tokens that compile to a slot-0 call to a runtime helper ──────────
RUNTIME_WORDS = {"emit", "swap", "0="}


# ── Token-level macros: expanded at compile time before any code-gen ──
MACROS: Dict[str, List[str]] = {
    "1+":     ["1", "+"],
    "1-":     ["0x3FFFF", "+"],         # -1 in 18-bit
    "-":      ["inv", "1", "+", "+"],   # a + (~b + 1)
    "or":     ["inv", "swap", "inv", "and", "inv"],
    "nip":    ["swap", "drop"],
    ">R":     ["push"],
    "R>":     ["pop"],
    "invert": ["inv"],
    "negate": ["inv", "1", "+"],
    "not":    ["inv"],                   # bit-not alias used by some forths
}


# ── Compile-time control-flow keywords ────────────────────────────────
CTRL_WORDS = {"if", "else", "then"}


@dataclass
class CompilerState:
    dictionary: Dict[str, List[str]] = field(default_factory=dict)


# ── Source-level tokenizer ────────────────────────────────────────────
def tokenize(source: str) -> List[str]:
    out: List[str] = []
    in_paren = 0
    for raw in source.splitlines():
        for tok in raw.split():
            if tok == "(":
                in_paren += 1
                continue
            if tok == ")":
                in_paren = max(0, in_paren - 1)
                continue
            if in_paren > 0:
                continue
            if tok == "\\":
                break
            out.append(tok)
    return out


def is_int_literal(tok: str) -> bool:
    try:
        parse_int_literal(tok)
        return True
    except ValueError:
        return False


def parse_int_literal(tok: str) -> int:
    return int(tok, 0)


def expand_macros(tokens: List[str]) -> List[str]:
    """Recursively expand MACROS entries. Built-in primitives, runtime
    words and control words are left alone."""
    out: List[str] = []
    for tok in tokens:
        if tok in MACROS:
            out.extend(expand_macros(MACROS[tok]))
        else:
            out.append(tok)
    return out


def _mask18(n: int) -> int:
    return n & 0x3FFFF


def _arith_shr18(n: int) -> int:
    n &= 0x3FFFF
    return ((n >> 1) | (0x20000 if (n & 0x20000) else 0)) & 0x3FFFF


def optimize_tokens(tokens: List[str]) -> List[str]:
    """Small compile-time stack evaluator for literal-only phrases.

    This is intentionally conservative: it only folds operations when the
    entire required stack slice is known at compile time, and flushes the
    virtual stack before any dynamic/control-flow word."""
    out: List[str] = []
    consts: List[int] = []

    def flush_consts() -> None:
        nonlocal consts
        out.extend(str(v) for v in consts)
        consts = []

    for tok in tokens:
        if is_int_literal(tok):
            consts.append(_mask18(parse_int_literal(tok)))
            continue

        if tok == "dup" and len(consts) >= 1:
            consts.append(consts[-1])
            continue
        if tok == "drop" and len(consts) >= 1:
            consts.pop()
            continue
        if tok == "swap" and len(consts) >= 2:
            consts[-2], consts[-1] = consts[-1], consts[-2]
            continue
        if tok == "over" and len(consts) >= 2:
            consts.append(consts[-2])
            continue

        if tok == "inv" and len(consts) >= 1:
            consts[-1] = _mask18(~consts[-1])
            continue
        if tok == "2*" and len(consts) >= 1:
            consts[-1] = _mask18(consts[-1] << 1)
            continue
        if tok == "2/" and len(consts) >= 1:
            consts[-1] = _arith_shr18(consts[-1])
            continue
        if tok == "0=" and len(consts) >= 1:
            consts[-1] = 0x3FFFF if consts[-1] == 0 else 0
            continue

        if tok in {"+", "and", "xor"} and len(consts) >= 2:
            b = consts.pop()
            a = consts.pop()
            if tok == "+":
                consts.append(_mask18(a + b))
            elif tok == "and":
                consts.append(_mask18(a & b))
            else:
                consts.append(_mask18(a ^ b))
            continue

        flush_consts()
        out.append(tok)

    flush_consts()
    return out


def split_definitions(state: CompilerState, tokens: List[str]) -> List[str]:
    body: List[str] = []
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t == ":":
            if i + 1 >= len(tokens):
                raise ValueError("`:` with no name after it")
            name = tokens[i + 1]
            j = i + 2
            def_body: List[str] = []
            while j < len(tokens) and tokens[j] != ";":
                def_body.append(tokens[j])
                j += 1
            if j >= len(tokens):
                raise ValueError(f"definition of `{name}` missing `;`")
            state.dictionary[name] = def_body
            i = j + 1
        elif t == ";":
            raise ValueError("`;` outside of definition")
        else:
            body.append(t)
            i += 1
    return body


# ── Body emitter ──────────────────────────────────────────────────────
class _BodyEmitter:
    """Turn a token list into a labelled asm sequence.

    Each "step" gets an auto-label `<prefix>_s<N>`. Every body word ends
    in an explicit slot-1 `jump:<next>` (or is a self-contained slot-0
    branch) so no implicit slot-3 ret fires inside a `:` body.
    """

    def __init__(self, prefix: str):
        self.prefix = prefix
        self.lines: List[str] = []
        self.step = 0
        self.gensym = 0
        self.if_stack: List[Dict[str, object]] = []

    def _label(self, n: int) -> str:
        return f"{self.prefix}_s{n}"

    def cur_label(self) -> str:
        return self._label(self.step)

    def next_label(self) -> str:
        return self._label(self.step + 1)

    def fresh_label(self, kind: str) -> str:
        n = self.gensym
        self.gensym += 1
        return f"{self.prefix}_{kind}{n}"

    def _emit_word(self, body: str) -> None:
        self.lines.append(f"{self.cur_label()}:")
        self.lines.append(f"    {body}")
        self.step += 1

    def _emit_lit_step(self, n: int) -> None:
        # `@p` reads M[P] as a literal and the slot-1 jump skips past
        # the literal to the next labelled step. The literal data word
        # consumes a memory cell but no logical step — it has no label.
        self.lines.append(f"{self.cur_label()}:")
        self.lines.append(f"    @p jump:{self.next_label()}")
        self.lines.append(f"    [0x{n:05x}]")
        self.step += 1

    def emit_label_only(self, name: str) -> None:
        # Bare label points to whatever we emit next (or the trailing
        # ret, if nothing else comes).
        self.lines.append(f"{name}:")

    # ── Token compilation ───────────────────────────────────────
    def emit_token(self, tok: str, state: CompilerState) -> None:
        if tok in CTRL_WORDS:
            getattr(self, f"_ctrl_{tok}")()
            return
        if is_int_literal(tok):
            self._emit_lit_step(parse_int_literal(tok) & 0x3FFFF)
            return
        if tok in PRIMITIVE_OPS:
            self._emit_word(f"{tok} jump:{self.next_label()}")
            return
        if tok in RUNTIME_WORDS:
            self._emit_word(f"call:_{_runtime_label(tok)}")
            return
        if tok in state.dictionary:
            self._emit_word(f"call:{tok}")
            return
        raise ValueError(f"unknown word `{tok}`")

    # ── Control-flow code-gen ───────────────────────────────────
    def _ctrl_if(self) -> None:
        skip_lbl = self.fresh_label("skip")
        then_lbl = self.fresh_label("then")
        # Slot-0 if: branches when T == 0; falls through when T != 0.
        # The fall-through path comes first and consumes the (nonzero)
        # flag; the skip path also has to drop its (zero) flag.
        self._emit_word(f"if:{skip_lbl}")
        self._emit_word(f"drop jump:{self.next_label()}")
        self.if_stack.append({"skip": skip_lbl, "then": then_lbl, "else_seen": False})

    def _ctrl_else(self) -> None:
        if not self.if_stack:
            raise ValueError("`else` without matching `if`")
        frame = self.if_stack[-1]
        if frame["else_seen"]:
            raise ValueError("double `else`")
        # End the true body, jump past the else body, drop in the false path.
        self._emit_word(f"jump:{frame['then']}")
        self.emit_label_only(str(frame["skip"]))
        self._emit_word(f"drop jump:{self.next_label()}")
        frame["else_seen"] = True

    def _ctrl_then(self) -> None:
        if not self.if_stack:
            raise ValueError("`then` without matching `if`")
        frame = self.if_stack.pop()
        if not frame["else_seen"]:
            # Synthesize an empty else: jump to then, drop on the false path.
            self._emit_word(f"jump:{frame['then']}")
            self.emit_label_only(str(frame["skip"]))
            self._emit_word(f"drop jump:{self.next_label()}")
        self.emit_label_only(str(frame["then"]))

    # ── Termination ─────────────────────────────────────────────
    def end_with_ret(self) -> None:
        # Final word for a `:` body: a stand-alone slot-0 ret.
        self.lines.append(f"{self.cur_label()}:")
        self.lines.append("    ;")
        self.step += 1

    def passthrough_label(self) -> str:
        """Label of the next emission slot — used by callers that want
        to splice further code on after the body."""
        return self.cur_label()


def _runtime_label(tok: str) -> str:
    """Map a runtime-word source token to its compiler label suffix."""
    return {"emit": "emit", "swap": "swap", "0=": "0eq"}[tok]


# ── Whole-program emit ────────────────────────────────────────────────
_RUNTIME_HELPERS = {
    "emit": [
        "# ── _emit ( c -- ) ───────────────────────────────────────",
        "_emit:",
        "    a! jump:_emit_poll",
        "_emit_poll:",
        "    @p b! jump:_emit_pollW2",
        "    [0x7F2]",
        "_emit_pollW2:",
        "    @b @p jump:_emit_pollW3",
        "    [2]",
        "_emit_pollW3:",
        "    and jump:_emit_check",
        "_emit_check:",
        "    if:_emit_drop_loop",
        "_emit_send:",
        "    drop jump:_emit_sendW1",
        "_emit_sendW1:",
        "    @p b! jump:_emit_sendW2",
        "    [0x7F1]",
        "_emit_sendW2:",
        "    a !b ;",
        "_emit_drop_loop:",
        "    drop jump:_emit_poll",
        "",
    ],
    "swap": [
        "# ── _swap ( a b -- b a ) — uses 0x37F as a one-cell scratch ──",
        "_swap:",
        "    @p b! jump:_swap_w1",
        "    [0x37F]",
        "_swap_w1:",
        "    !b a! jump:_swap_w2",
        "_swap_w2:",
        "    @b a ;",
        "",
    ],
    "0=": [
        "# ── _0eq ( n -- f ) — f = -1 if n==0 else 0 ─────────────",
        "_0eq:",
        "    if:_0eq_zero",
        "_0eq_nonzero:",
        "    drop @p jump:_0eq_done",
        "    [0x00000]",
        "_0eq_zero:",
        "    drop @p jump:_0eq_done",
        "    [0x3FFFF]",
        "_0eq_done:",
        "    ;",
        "",
    ],
}


def compile_program(state: CompilerState, body_tokens: List[str]) -> str:
    body_tokens = optimize_tokens(expand_macros(body_tokens))
    user_defs = {
        name: optimize_tokens(expand_macros(body))
        for name, body in state.dictionary.items()
    }
    needed_helpers = {
        tok
        for toks in [body_tokens, *user_defs.values()]
        for tok in toks
        if tok in RUNTIME_WORDS
    }

    lines: List[str] = []
    add = lines.append
    extend = lines.extend

    add("# auto-generated by forth_compile.py — fresh image per REPL line")
    add("#origin 0x000")
    add("entry:")
    add("    jump:main")
    add("")

    for name in ("emit", "swap", "0="):
        if name in needed_helpers:
            extend(_RUNTIME_HELPERS[name])

    # Top-level body (no terminating ret — splice the halt sequence
    # onto the end). The halt sequence writes the magic cookie 1 to
    # the "done" RAM cell at 0x37E — the sim back-end watches for that
    # write to know UART_TX output has stopped, and the hardware
    # back-end ignores it (it falls back to an idle-timeout detector).
    # Using a RAM-write marker instead of an in-band UART byte means
    # any byte value (including 0x04) is allowed in user output.
    add("main:")
    main_emitter = _BodyEmitter("main")
    for tok in body_tokens:
        main_emitter.emit_token(tok, state)
    if main_emitter.if_stack:
        raise ValueError("unbalanced `if`/`then` in top-level body")
    extend(main_emitter.lines)
    s0 = main_emitter.cur_label()
    s1 = main_emitter._label(main_emitter.step + 1)
    s2 = main_emitter._label(main_emitter.step + 2)
    s3 = main_emitter._label(main_emitter.step + 3)
    s4 = main_emitter._label(main_emitter.step + 4)
    add(f"{s0}:")
    add(f"    @p jump:{s1}")
    add("    [0x37E]")               # done-marker RAM addr
    add(f"{s1}:")
    add(f"    a! jump:{s2}")          # A := 0x37E
    add(f"{s2}:")
    add(f"    @p jump:{s3}")
    add("    [0x00001]")              # cookie value
    add(f"{s3}:")
    add(f"    ! jump:{s4}")           # store cookie at A
    add(f"{s4}:")
    # Hand control back to INST_PORT (DB001 §3.3.2 port-execution) so the
    # host can stream a fresh program over UART without re-flashing the
    # bitstream — same trick ackermann.f18a uses at end-of-run.
    add("    jump:0x7F9")
    add("")

    # User word definitions.
    for name, body in user_defs.items():
        add(f"{name}:")
        ue = _BodyEmitter(f"_def_{name}")
        for tok in body:
            ue.emit_token(tok, state)
        if ue.if_stack:
            raise ValueError(f"unbalanced `if`/`then` in `{name}`")
        ue.end_with_ret()
        extend(ue.lines)
        add("")

    return "\n".join(lines) + "\n"


def compile_line(state: CompilerState, source: str) -> str | None:
    tokens = tokenize(source)
    body = split_definitions(state, tokens)
    if not body:
        return None
    return compile_program(state, body)
