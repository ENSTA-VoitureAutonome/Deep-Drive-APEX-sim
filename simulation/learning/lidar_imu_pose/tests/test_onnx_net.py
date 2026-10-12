"""The network exported to ONNX (``onnx_net.py``) gives the PyTorch outputs, scan after scan, with the
state carried over. Needs the v5 checkpoint and the replay file (skipped otherwise)."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest
import torch

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

pytest.importorskip("onnxruntime")
pytest.importorskip("onnx")
CKPT = HERE.parent / "outputs" / "real2sim_v5_10hz" / "hybrid_submap_v4" / "best_model.pt"
REPLAY = HERE.parent / "outputs" / "pi4_budget" / "replay_validation.npz"
pytestmark = pytest.mark.skipif(not (CKPT.exists() and REPLAY.exists()), reason="checkpoint or replay not available")


def _stream_matches(ckpt: Path, tmp_path: Path) -> Path:
    """Export ``ckpt`` and compare ONNX Runtime with PyTorch over 60 replayed scans; returns the .onnx path."""
    import latency_profile as lp
    import onnx_net

    rp = lp.Replay(REPLAY)
    live = lp.new_live(ckpt, rp, 0)
    batches: list[dict[str, torch.Tensor]] = []
    orig = live.model.forward

    def grab(batch, state):  # noqa: ANN001, ANN202
        batches.append({k: v.clone() for k, v in batch.items()})
        return orig(batch, state)

    live.model.forward = grab
    lp.replay_run(live, rp.run(0), max_scans=60)
    path = onnx_net.export(ckpt, batches[5], live.model.initial_state(1, torch.device("cpu")), tmp_path / "net.onnx")
    net = onnx_net.OnnxStreamingNet(ckpt, True, int(live.model.hidden), path=path)
    st_t = live.model.initial_state(1, torch.device("cpu"))
    st_o = net.initial_state(1, torch.device("cpu"))
    with torch.no_grad():
        for b in batches:
            out_t, st_t = orig(b, st_t)
            out_o, st_o = net(b, st_o)
            for k in onnx_net.OUT_KEYS:
                assert torch.allclose(out_t[k], out_o[k], atol=1e-5, rtol=1e-4), k
            assert torch.allclose(st_t["h"], st_o["h"], atol=1e-5)
    return path


def test_onnx_matches_torch_over_a_stream(tmp_path: Path) -> None:
    _stream_matches(CKPT, tmp_path)


def test_window_imu_encoder_exports_without_the_loop(tmp_path: Path) -> None:
    """v6's layout (v5 with ``WindowImuEncoder``, untrained here) exports as is, with a fraction of the nodes."""
    import distill
    import onnx
    from streaming_model import load_checkpoint

    teacher, ck = load_checkpoint(CKPT)
    ckpt = tmp_path / "student.pt"
    torch.save({"model": distill.student_from(teacher).state_dict(), "config": {**ck["config"], "imu_encoder": "window"}}, ckpt)
    graph = onnx.load(str(_stream_matches(ckpt, tmp_path))).graph
    assert len(graph.node) < 1000  # v5: ~3000, 40 unrolled GRU steps
