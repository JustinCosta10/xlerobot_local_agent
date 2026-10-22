from types import SimpleNamespace

import numpy as np
import pytest
from std_msgs.msg import Header

from xlerobot_perception.processing import messages_from_arrays
from xlerobot_perception.vocabulary import DEFAULT_CLASSES, PromptModel, Vocabulary, validate_classes


def test_default_targets_and_stable_mask_ids_after_reordering():
    vocabulary = Vocabulary()
    vocabulary.activate(DEFAULT_CLASSES)
    vocabulary.activate(["mustard bottle", "mug", "book"])
    assert vocabulary.class_ids == [5, 1, 8]
    detections, labels = messages_from_arrays(
        Header(), (2, 2), [[0, 0, 2, 2]], [.9], [0], np.ones((1, 2, 2)),
        class_ids=vocabulary.class_ids)
    assert (labels == 5).all()
    assert detections.detections[0].results[0].hypothesis.class_id == "5"
    vocabulary.activate(["mug"])
    assert vocabulary.class_ids == [1] and vocabulary.labels["5"] == "mustard bottle"


@pytest.mark.parametrize("names", [[], [" "], ["red cube", " RED  CUBE "], ["x" * 121],
                                  ["x\x00y"], [str(i) for i in range(129)]])
def test_invalid_vocabulary(names):
    with pytest.raises(ValueError):
        validate_classes(names)


def test_class_normalization_and_idempotent_revision():
    assert validate_classes([" RED   Cube "]) == ["red cube"]
    vocabulary = Vocabulary()
    vocabulary.activate(["red cube"])
    vocabulary.activate(["red cube"])
    assert vocabulary.revision == 1


def test_embedding_cache_and_failure_keeps_previous_targets():
    from collections import OrderedDict
    encoded, applied = [], []

    def encode(names, cache_clip_model):
        assert cache_clip_model
        encoded.append(tuple(names))
        if names == ["error"]:
            raise RuntimeError("encoder failed")
        return object()

    prompt = PromptModel.__new__(PromptModel)
    prompt.model = SimpleNamespace(
        model=SimpleNamespace(get_text_pe=encode),
        set_classes=lambda names, embedding: applied.append((list(names), embedding)))
    prompt.active, prompt.embeddings, prompt.cache = [], None, OrderedDict()
    prompt.set_classes(["red cube"])
    first_embedding = prompt.embeddings
    prompt.set_classes(["blue cube"])
    prompt.set_classes(["red cube"])
    assert prompt.embeddings is first_embedding
    assert encoded == [("red cube",), ("blue cube",)]
    with pytest.raises(RuntimeError, match="encoder failed"):
        prompt.set_classes(["error"])
    assert prompt.active == ["red cube"] and prompt.embeddings is first_embedding
    assert applied[-1] == (["red cube"], first_embedding)
