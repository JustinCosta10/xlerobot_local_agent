"""Validated vocabularies and session-stable semantic class identifiers."""

from collections import OrderedDict


DEFAULT_CLASSES = [f"{color} cube" for color in
                   ("red", "orange", "yellow", "green", "blue", "purple", "black", "white")]


def validate_classes(classes):
    if not 1 <= len(classes) <= 128:
        raise ValueError("Provide between 1 and 128 class names")
    names = []
    for value in classes:
        if not isinstance(value, str):
            raise ValueError("Class names must be strings")
        name = " ".join(value.split()).casefold()
        if not name or len(name) > 120 or not name.isprintable():
            raise ValueError("Class names must contain 1–120 printable characters")
        if name in names:
            raise ValueError(f"Duplicate class name: {name}")
        names.append(name)
    return names


class Vocabulary:
    """Only the inference worker writes this state after startup."""

    def __init__(self):
        self.labels = {"0": "unknown"}
        self.ids_by_name = {}
        self.active = []
        self.class_ids = []
        self.revision = 0

    def check_capacity(self, names):
        new = len(set(names) - self.ids_by_name.keys())
        if len(self.ids_by_name) + new > 65535:
            raise ValueError("Semantic class ID space exhausted; restart the node")

    def activate(self, names):
        self.check_capacity(names)
        if self.active == names:
            return
        for name in names:
            if name not in self.ids_by_name:
                label = len(self.ids_by_name) + 1
                self.ids_by_name[name] = label
                self.labels[str(label)] = name
        self.active = list(names)
        self.class_ids = [self.ids_by_name[name] for name in names]
        self.revision += 1


class PromptModel:
    """Own the promptable model; cache text encodings for repeated vocabularies."""

    def __init__(self, path, encoder_path, device):
        from pathlib import Path
        import torch
        from ultralytics import YOLOE
        from ultralytics.nn.text_model import MobileCLIPTS

        for file in (path, encoder_path):
            if not Path(file).is_file():
                raise FileNotFoundError(f"YOLOE asset missing: {file}. Run scripts/install_yoloe.sh.")
        torch.set_num_threads(2)
        self.model = YOLOE(path)
        if hasattr(self.model.model.model[-1], "lrpc"):
            raise ValueError("Live vocabulary requires yoloe-11s-seg.pt, not the prompt-free *-pf.pt model")
        self.model.to(device if device == "cpu" else f"cuda:{device}" if str(device).isdigit() else device)
        # Supply an absolute local encoder path: no implicit download or CWD lookup.
        self.model.model.clip_model = MobileCLIPTS(
            device=next(self.model.model.parameters()).device, weight=encoder_path)
        self.cache = OrderedDict()
        self.active = []
        self.embeddings = None

    def set_classes(self, names):
        if names == self.active:
            return
        key = tuple(names)
        embeddings = self.cache.get(key)
        if embeddings is None:
            embeddings = self.model.model.get_text_pe(names, cache_clip_model=True)
        # Encoding/validation happens before the installed model state is changed.
        try:
            self.model.set_classes(names, embeddings)
        except Exception:
            if self.active:
                self.model.set_classes(self.active, self.embeddings)
            raise
        self.active, self.embeddings = list(names), embeddings
        self.cache[key] = embeddings
        self.cache.move_to_end(key)
        if len(self.cache) > 32:
            self.cache.popitem(last=False)

    def predict(self, *args, **kwargs):
        return self.model.predict(*args, **kwargs)
