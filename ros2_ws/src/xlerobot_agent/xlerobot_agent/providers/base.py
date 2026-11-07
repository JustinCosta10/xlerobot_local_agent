"""Portable visible conversation, with opaque provider continuation kept in memory."""

from dataclasses import dataclass, field
from typing import Protocol


@dataclass
class ToolCall:
    call_id: str
    name: str
    arguments: str


@dataclass
class Turn:
    role: str
    text: str = ""
    calls: list[ToolCall] = field(default_factory=list)
    call_id: str = ""
    # Not logged: e.g. Responses reasoning/encrypted continuation items.
    provider_items: list[dict] = field(default_factory=list)


class Provider(Protocol):
    def next_turn(self, conversation: list[Turn], tools: list[dict], instructions: str) -> Turn:
        ...
