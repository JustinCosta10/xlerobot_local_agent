"""Responses REST adapter, using stdlib HTTP so ROS/YOLOE need no SDK upgrade."""

import json
import os
import re
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

from .base import ToolCall, Turn


class ProviderError(RuntimeError):
    pass


class OpenAIProvider:
    def __init__(self, model, *, endpoint="https://api.openai.com/v1/responses",
                 timeout_s=60., max_output_tokens=4096, reasoning_effort="low",
                 api_key=None, transport=None):
        self.model = model
        self.endpoint = endpoint
        self.timeout_s = timeout_s
        self.max_output_tokens = max_output_tokens
        self.reasoning_effort = reasoning_effort
        self.api_key = api_key if api_key is not None else os.environ.get("OPENAI_API_KEY", "")
        self.transport = transport or self._request
        if transport is None and not self.api_key:
            raise ProviderError("Set OPENAI_API_KEY in your terminal, or use --demo for an offline test.")
        if urlparse(endpoint).scheme != "https" and transport is None:
            raise ProviderError("The OpenAI endpoint must use HTTPS.")

    def _request(self, payload):
        request = Request(self.endpoint, data=json.dumps(payload).encode(), headers={
            "Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                return json.load(response)
        except HTTPError as error:
            # Do not echo arbitrary server bodies, URLs, or request headers into traces.
            code = "request_failed"
            try:
                candidate = json.loads(error.read()).get("error", {}).get("code")
                if isinstance(candidate, str) and re.fullmatch(r"[a-zA-Z0-9_]{1,80}", candidate):
                    code = candidate
            except (ValueError, AttributeError):
                pass
            raise ProviderError(f"OpenAI HTTP {error.code} ({code}); check model access, credits, and key.") from None
        except (URLError, TimeoutError, OSError):
            raise ProviderError("OpenAI connection failed or timed out; no tool calls from this request executed.") from None
        except ValueError:
            raise ProviderError("OpenAI returned invalid JSON.") from None

    def next_turn(self, conversation, tools, instructions):
        inputs = []
        for turn in conversation:
            if turn.role == "tool":
                inputs.append({"type": "function_call_output", "call_id": turn.call_id,
                               "output": turn.text})
            elif turn.role == "assistant" and turn.provider_items:
                inputs.extend(turn.provider_items)
            else:
                inputs.append({"role": turn.role, "content": turn.text})
        payload = dict(model=self.model, instructions=instructions, input=inputs,
                       tools=[dict(type="function", strict=True, **tool) for tool in tools],
                       parallel_tool_calls=False, store=False,
                       include=["reasoning.encrypted_content"],
                       max_output_tokens=self.max_output_tokens)
        if self.reasoning_effort:
            payload["reasoning"] = {"effort": self.reasoning_effort}
        response = self.transport(payload)
        if response.get("status") != "completed":
            raise ProviderError("Model response did not complete; no partial function calls executed.")
        output = response.get("output", [])
        calls, visible = [], []
        try:
            for item in output:
                if item["type"] == "function_call":
                    if not all(isinstance(item.get(k), str) and item[k]
                               for k in ("call_id", "name", "arguments")):
                        raise ValueError("Invalid function call")
                    calls.append(ToolCall(item["call_id"], item["name"], item["arguments"]))
                elif item["type"] == "message":
                    for part in item.get("content", []):
                        if part["type"] == "output_text":
                            visible.append(part["text"])
                        elif part["type"] == "refusal":
                            visible.append(part["refusal"])
            if len({call.call_id for call in calls}) != len(calls):
                raise ValueError("Duplicate call IDs")
            if not calls and not visible:
                raise ValueError("Empty response")
        except (KeyError, TypeError, ValueError):
            raise ProviderError("Model response contained invalid or empty output; nothing dispatched.") from None
        return Turn("assistant", text="\n".join(visible), calls=calls, provider_items=output)
