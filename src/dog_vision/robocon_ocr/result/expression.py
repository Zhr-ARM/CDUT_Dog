from __future__ import annotations

from dataclasses import dataclass
import re


ALLOWED_CHARS = set("0123456789+-×÷()= ")
SYMBOL_REPLACEMENTS = {
    "x": "×",
    "X": "×",
    "*": "×",
    "✖": "×",
    "✖️": "×",
    "／": "÷",
    "/": "÷",
    "—": "-",
    "−": "-",
    "（": "(",
    "）": ")",
    "[": "(",
    "]": ")",
}
LATEX_REPLACEMENTS = {
    r"\times": "×",
    r"\cdot": "×",
    r"\div": "÷",
    r"\left(": "(",
    r"\right)": ")",
    "[": "(",
    "]": ")",
    "（": "(",
    "）": ")",
}
ARITHMETIC_CHARS_NO_SPACE = set("0123456789+-×÷()=")
UNSUPPORTED_LATEX_COMMANDS = {
    "frac",
    "sqrt",
    "sum",
    "int",
    "prod",
    "lim",
    "sin",
    "cos",
    "tan",
    "log",
}


@dataclass(slots=True)
class ParsedExpression:
    normalized_text: str
    expression: str
    answer: int | None
    is_valid: bool
    error: str | None = None


@dataclass(slots=True)
class RepairCandidate:
    expression: str
    cost: int


def normalize_ocr_text(text: str) -> str:
    normalized = text.strip()
    normalized = normalized.replace("$", "")
    normalized = normalized.replace("\n", "")
    normalized = normalized.replace("\r", "")
    for src, dst in LATEX_REPLACEMENTS.items():
        normalized = normalized.replace(src, dst)
    normalized = re.sub(r"\\[a-zA-Z]+\*?", " ", normalized)
    normalized = re.sub(r"[{}&,_^]", " ", normalized)
    for src, dst in SYMBOL_REPLACEMENTS.items():
        normalized = normalized.replace(src, dst)
    normalized = re.sub(r"\s+", "", normalized)
    extracted = extract_arithmetic_candidate(normalized)
    if extracted is not None:
        return extracted
    return normalized


def extract_arithmetic_candidate(text: str) -> str | None:
    candidates = re.findall(r"[0-9+\-×÷()=]+", text)
    if not candidates:
        return None

    best: str | None = None
    best_score = (-1, -1, -1)
    for candidate in candidates:
        digit_count = sum(ch.isdigit() for ch in candidate)
        operator_count = sum(ch in "+-×÷" for ch in candidate)
        if digit_count < 2 or operator_count < 1:
            continue
        score = (len(candidate), digit_count, operator_count)
        if score > best_score:
            best = candidate
            best_score = score
    return best


def validate_ocr_text(text: str) -> str | None:
    if not text:
        return None
    invalid_chars = [ch for ch in text if ch not in ALLOWED_CHARS]
    if invalid_chars:
        return "unsupported symbol outside arithmetic charset"
    return None


def contains_unsupported_latex(text: str) -> bool:
    commands = re.findall(r"\\([a-zA-Z]+)\*?", text)
    return any(command in UNSUPPORTED_LATEX_COMMANDS for command in commands)


def to_expression_only(text: str) -> str:
    cleaned = normalize_ocr_text(text)
    cleaned = "".join(ch for ch in cleaned if ch in ALLOWED_CHARS)
    cleaned = cleaned.replace(" ", "")
    if "=" in cleaned:
        cleaned = cleaned.split("=", 1)[0]
    return cleaned


def tokenize(expression: str) -> list[str]:
    tokens: list[str] = []
    number = []
    for ch in expression:
        if ch.isdigit():
            number.append(ch)
            continue
        if number:
            tokens.append("".join(number))
            number.clear()
        tokens.append(ch)
    if number:
        tokens.append("".join(number))
    return tokens


def _try_parse(expression: str) -> tuple[int | None, str | None]:
    try:
        tokens = tokenize(expression)
        return ExpressionParser(tokens).parse(), None
    except ValueError as exc:
        return None, str(exc)


def _collapse_duplicate_operators(expression: str) -> RepairCandidate | None:
    operators = {"+", "-", "×", "÷"}
    chars: list[str] = []
    cost = 0
    changed = False

    for ch in expression:
        if chars and ch in operators and chars[-1] == ch:
            changed = True
            cost += 1
            continue
        chars.append(ch)

    if not changed:
        return None
    return RepairCandidate("".join(chars), cost)


def _replace_internal_equals(expression: str) -> list[RepairCandidate]:
    candidates: list[RepairCandidate] = []
    end = len(expression) - 1 if expression.endswith("=") else len(expression)
    for index, ch in enumerate(expression[:end]):
        if ch != "=":
            continue
        for replacement in ("÷", "×"):
            candidates.append(
                RepairCandidate(
                    expression[:index] + replacement + expression[index + 1 :],
                    1,
                )
            )
    return candidates


def _sanitize_parentheses(expression: str) -> RepairCandidate | None:
    chars: list[str] = []
    balance = 0
    removed = 0

    for ch in expression:
        if ch == "(":
            balance += 1
            chars.append(ch)
            continue
        if ch == ")":
            if balance == 0:
                removed += 1
                continue
            balance -= 1
            chars.append(ch)
            continue
        chars.append(ch)

    if removed == 0 and balance == 0:
        return None
    return RepairCandidate("".join(chars) + (")" * balance), removed + balance)


def _repair_expression(expression: str) -> ParsedExpression | None:
    pending = [RepairCandidate(expression, 0)]
    seen: dict[str, int] = {expression: 0}
    valid: list[tuple[int, int, ParsedExpression]] = []

    while pending:
        current = pending.pop(0)
        if current.cost > 2:
            continue

        value, error = _try_parse(current.expression)
        if error is None:
            valid.append(
                (
                    current.cost,
                    -len(current.expression),
                    ParsedExpression(
                        normalized_text=current.expression,
                        expression=current.expression,
                        answer=value,
                        is_valid=True,
                    ),
                )
            )
            continue

        derived: list[RepairCandidate] = []
        duplicate = _collapse_duplicate_operators(current.expression)
        if duplicate is not None:
            derived.append(RepairCandidate(duplicate.expression, current.cost + duplicate.cost))
        parenthesized = _sanitize_parentheses(current.expression)
        if parenthesized is not None:
            derived.append(
                RepairCandidate(parenthesized.expression, current.cost + parenthesized.cost)
            )
        for candidate in _replace_internal_equals(current.expression):
            derived.append(RepairCandidate(candidate.expression, current.cost + candidate.cost))

        for candidate in derived:
            if candidate.cost > 2:
                continue
            previous_cost = seen.get(candidate.expression)
            if previous_cost is not None and previous_cost <= candidate.cost:
                continue
            seen[candidate.expression] = candidate.cost
            pending.append(candidate)

    if not valid:
        return None

    valid.sort(key=lambda item: (item[0], item[1]))
    return valid[0][2]


class ExpressionParser:
    def __init__(self, tokens: list[str]) -> None:
        self.tokens = tokens
        self.index = 0

    def current(self) -> str | None:
        if self.index >= len(self.tokens):
            return None
        return self.tokens[self.index]

    def consume(self, expected: str | None = None) -> str:
        token = self.current()
        if token is None:
            raise ValueError("unexpected end of expression")
        if expected is not None and token != expected:
            raise ValueError(f"expected {expected!r}, got {token!r}")
        self.index += 1
        return token

    def parse(self) -> int:
        value = self.parse_add_sub()
        if self.current() is not None:
            raise ValueError(f"unexpected token {self.current()!r}")
        return value

    def parse_add_sub(self) -> int:
        value = self.parse_mul_div()
        while self.current() in {"+", "-"}:
            op = self.consume()
            rhs = self.parse_mul_div()
            value = value + rhs if op == "+" else value - rhs
        return value

    def parse_mul_div(self) -> int:
        value = self.parse_primary()
        while self.current() in {"×", "÷"}:
            op = self.consume()
            rhs = self.parse_primary()
            if op == "×":
                value *= rhs
            else:
                if rhs == 0:
                    raise ValueError("division by zero")
                if value % rhs != 0:
                    raise ValueError("non-integer division")
                value //= rhs
        return value

    def parse_primary(self) -> int:
        token = self.current()
        if token is None:
            raise ValueError("unexpected end of expression")
        if token == "(":
            self.consume("(")
            value = self.parse_add_sub()
            self.consume(")")
            return value
        if token.isdigit():
            return int(self.consume())
        raise ValueError(f"unexpected token {token!r}")


def parse_expression(text: str) -> ParsedExpression:
    normalized = normalize_ocr_text(text)
    if contains_unsupported_latex(text):
        extracted = extract_arithmetic_candidate(normalized)
        if extracted is None:
            return ParsedExpression(
                normalized_text=text.strip(),
                expression="",
                answer=None,
                is_valid=False,
                error="unsupported symbol outside arithmetic charset",
            )
        normalized = extracted
    validation_error = validate_ocr_text(normalized)
    if validation_error is not None:
        return ParsedExpression(
            normalized_text=normalized,
            expression="",
            answer=None,
            is_valid=False,
            error=validation_error,
        )
    expression = to_expression_only(normalized)

    if not expression:
        return ParsedExpression(
            normalized_text=normalized,
            expression="",
            answer=None,
            is_valid=False,
            error="empty expression",
        )

    answer, error = _try_parse(expression)
    if error is not None:
        repaired = _repair_expression(expression)
        if repaired is not None:
            repaired.normalized_text = normalized
            return repaired
        return ParsedExpression(
            normalized_text=normalized,
            expression=expression,
            answer=None,
            is_valid=False,
            error=error,
        )

    return ParsedExpression(
        normalized_text=normalized,
        expression=expression,
        answer=answer,
        is_valid=True,
    )
