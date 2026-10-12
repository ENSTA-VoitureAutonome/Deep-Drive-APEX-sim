"""The streaming network run by ONNX Runtime instead of PyTorch (same weights, no retraining).

On the Pi 4 the network's cost is mostly fixed: its ~417 top-level torch calls per scan cost
~36 ms of dispatch at any parameter count (``pi4_budget.py``, README §10-11), against ~9 ms for
the weights of v5. ONNX Runtime runs the same graph as one compiled call.

  export    the checkpoint's ``StreamingPoseNet`` with one scan step (batch 1, T = 1, the IMU
            zero-padded to ``MAX_IMU`` as ``LiveOdometry`` builds it), traced on a real batch
            (a GRU IMU encoder, up to v5, as a masked loop; the window perceptron of v6 as is).
            Written next to the checkpoint (``<checkpoint>.onnx``) with the input / output names
            below; ``check`` compares it with PyTorch on a whole replayed run.
  OnnxStreamingNet
            drop-in for the model in ``LiveOdometry`` (``net_backend="onnx"``): same call
            ``out, state = net(batch, state)`` with torch tensors in and out.

  python onnx_net.py export  --checkpoint ../outputs/real2sim_v5_10hz/hybrid_submap_v4/best_model.pt
  python onnx_net.py check   --checkpoint ... --replay ../outputs/pi4_budget/replay_validation.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any

import numpy as np
import torch
from torch import Tensor, nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

BATCH_KEYS = ("prev_ranges", "prev_valid", "cur_ranges", "cur_valid", "w_prev", "w_cur", "time_increment", "imu", "imu_lengths", "accel",
              "accel_std", "gyro_integral", "dt", "prof_prev", "prof_cur", "dt_next", "beam_frac", "icp_pred", "icp_feat")
STATE_KEYS = ("h", "v", "bias", "age")
OUT_KEYS = ("delta", "logvar", "velocity", "bias")


def onnx_path(checkpoint: str | Path) -> Path:
    return Path(checkpoint).with_suffix(".onnx")


class _GruCell(nn.Module):
    """``nn.GRUCell`` written out with the same weights (its ONNX export fails on ``unsafe_chunk``).

    r = sigmoid(W_ir x + b_ir + W_hr h + b_hr), z = sigmoid(W_iz x + b_iz + W_hz h + b_hz),
    n = tanh(W_in x + b_in + r (W_hn h + b_hn)), h' = (1 - z) n + z h (gate order r, z, n)."""

    def __init__(self, cell: nn.GRUCell) -> None:
        super().__init__()
        self.cell, self.size = cell, cell.hidden_size

    def forward(self, x: Tensor, h: Tensor) -> Tensor:
        c, n = self.cell, self.size
        gi = nn.functional.linear(x, c.weight_ih, c.bias_ih)
        gh = nn.functional.linear(h, c.weight_hh, c.bias_hh)
        r = torch.sigmoid(gi[:, :n] + gh[:, :n])
        z = torch.sigmoid(gi[:, n:2 * n] + gh[:, n:2 * n])
        cand = torch.tanh(gi[:, 2 * n:] + r * gh[:, 2 * n:])
        return (1.0 - z) * cand + z * h


class _ImuEncoder(nn.Module):
    """``train_pose_fusion.ImuEncoder`` without ``pack_padded_sequence`` (not exportable): the GRU steps
    over the ``MAX_IMU`` zero-padded samples and keeps its state past each window's length, which is
    the final state the packed sequence returns."""

    def __init__(self, enc: nn.Module) -> None:
        super().__init__()
        self.temporal, gru = enc.temporal, enc.gru
        cell = nn.GRUCell(gru.input_size, gru.hidden_size)
        with torch.no_grad():
            for name in ("weight_ih", "weight_hh", "bias_ih", "bias_hh"):
                getattr(cell, name).copy_(getattr(gru, f"{name}_l0"))
        self.cell = _GruCell(cell)

    def forward(self, imu: Tensor, lengths: Tensor) -> Tensor:
        b, t, samples, channels = imu.shape
        features = self.temporal(imu.reshape(b * t, samples, channels).transpose(1, 2)).transpose(1, 2)
        flat = lengths.reshape(-1, 1)
        h = features.new_zeros(b * t, self.cell.size)
        for k in range(samples):
            keep = (flat > k).to(features.dtype)
            h = keep * self.cell(features[:, k] * keep, h) + (1.0 - keep) * h
        return h.reshape(b, t, -1)


class _Step(nn.Module):
    """``StreamingPoseNet.forward`` on positional tensors (ONNX has no dicts)."""

    def __init__(self, model: nn.Module, keys: tuple[str, ...]) -> None:
        super().__init__()
        self.model, self.keys = model, keys

    def forward(self, *args: Tensor) -> tuple[Tensor, ...]:
        n = len(self.keys)
        batch = dict(zip(self.keys, args[:n]))
        state = dict(zip(STATE_KEYS, args[n:]))
        out, st = self.model(batch, state)
        return (*(out[k] for k in OUT_KEYS), *(st[k] for k in STATE_KEYS))


def export(checkpoint: str | Path, batch: dict[str, Tensor], state: dict[str, Tensor], path: Path | None = None) -> Path:
    """Trace the network on one real ``LiveOdometry`` batch and write the ONNX file."""
    from streaming_model import load_checkpoint  # noqa: PLC0415

    model, _ = load_checkpoint(checkpoint, "cpu")
    model.cells = nn.ModuleList([_GruCell(c) for c in model.cells])
    if model.imu_encoder_type == "gru":  # the window perceptron of v6 exports as it is
        model.imu_encoder = _ImuEncoder(model.imu_encoder)
    keys = tuple(k for k in BATCH_KEYS if k in batch)
    args = tuple(batch[k].cpu() for k in keys) + tuple(state[k].cpu() for k in STATE_KEYS)
    path = path or onnx_path(checkpoint)
    with torch.no_grad():
        torch.onnx.export(_Step(model, keys), args, str(path), input_names=[*keys, *(f"state_{k}" for k in STATE_KEYS)],
                          output_names=[*OUT_KEYS, *(f"next_{k}" for k in STATE_KEYS)], opset_version=17, dynamo=False)
    _bool_where_as_uint8(path)
    return path


def _bool_where_as_uint8(path: Path) -> None:
    """ONNX Runtime has no CPU ``Where`` for booleans (the valid masks of ``rotate_scan``): run those as
    uint8 between two casts, which is exact for 0 / 1."""
    import onnx  # noqa: PLC0415
    from onnx import TensorProto, helper, shape_inference  # noqa: PLC0415

    m = shape_inference.infer_shapes(onnx.load(str(path)))
    types = {vi.name: vi.type.tensor_type.elem_type for vi in [*m.graph.value_info, *m.graph.input, *m.graph.output]}
    nodes = []
    for n in m.graph.node:
        if n.op_type == "Where" and types.get(n.output[0]) == TensorProto.BOOL:
            x8, y8, o8 = (f"{n.name}_{s}_u8" for s in ("x", "y", "out"))
            nodes += [helper.make_node("Cast", [n.input[1]], [x8], to=TensorProto.UINT8), helper.make_node("Cast", [n.input[2]], [y8], to=TensorProto.UINT8),
                      helper.make_node("Where", [n.input[0], x8, y8], [o8], name=n.name), helper.make_node("Cast", [o8], [n.output[0]], to=TensorProto.BOOL)]
        else:
            nodes.append(n)
    del m.graph.node[:]
    m.graph.node.extend(nodes)
    onnx.checker.check_model(m)
    onnx.save(m, str(path))


class OnnxStreamingNet:
    """The exported network behind the ``StreamingPoseNet`` interface ``LiveOdometry`` uses."""

    def __init__(self, checkpoint: str | Path, hybrid: bool, hidden: int, threads: int = 1, path: Path | None = None) -> None:
        import onnxruntime as ort  # noqa: PLC0415

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads  # the budget assumes one core for the whole estimator
        opts.inter_op_num_threads = 1
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(str(path or onnx_path(checkpoint)), opts, providers=["CPUExecutionProvider"])
        self.inputs = [i.name for i in self.session.get_inputs()]
        self.hybrid, self.hidden = hybrid, hidden

    def initial_state(self, lanes: int, device: torch.device) -> dict[str, Tensor]:
        return {"h": torch.zeros(2, lanes, self.hidden), "v": torch.zeros(lanes, 2), "bias": torch.zeros(lanes, 3), "age": torch.zeros(lanes)}

    def __call__(self, batch: dict[str, Tensor], state: dict[str, Tensor]) -> tuple[dict[str, Tensor], dict[str, Tensor]]:
        return self.forward(batch, state)

    def forward(self, batch: dict[str, Tensor], state: dict[str, Tensor]) -> tuple[dict[str, Tensor], dict[str, Tensor]]:
        feed = {}
        for name in self.inputs:
            t = state[name[6:]] if name.startswith("state_") else batch[name]
            feed[name] = t.detach().cpu().numpy()
        res = self.session.run(None, feed)
        out = {k: torch.from_numpy(res[i]) for i, k in enumerate(OUT_KEYS)}
        st = {k: torch.from_numpy(res[len(OUT_KEYS) + i]) for i, k in enumerate(STATE_KEYS)}
        return out, st


# --------------------------------------------------------------------- CLI
def _capture(checkpoint: str, replay: Path, run: int = 0, at: int = 30) -> tuple[dict[str, Tensor], dict[str, Tensor]]:
    """One real batch and state from a replayed run (``latency_profile.Replay``)."""
    import latency_profile as lp  # noqa: PLC0415

    rp = lp.Replay(replay)
    live = lp.new_live(Path(checkpoint), rp, run)
    got: dict[str, Any] = {}
    orig = live.model.forward

    def grab(batch: dict[str, Tensor], state: dict[str, Tensor]) -> Any:  # noqa: ANN401
        if live.scans == at and not got:
            got["batch"] = {k: v.clone() for k, v in batch.items()}
            got["state"] = {k: v.clone() for k, v in state.items()}
        return orig(batch, state)

    live.model.forward = grab
    lp.replay_run(live, rp.run(run), max_scans=at + 2)
    return got["batch"], got["state"]


def cmd_export(args: argparse.Namespace) -> None:
    batch, state = _capture(args.checkpoint, Path(args.replay))
    print("wrote", export(args.checkpoint, batch, state))


def cmd_check(args: argparse.Namespace) -> None:
    """PyTorch and ONNX Runtime on the same replayed run: per-scan outputs and the integrated pose."""
    import latency_profile as lp  # noqa: PLC0415

    rp = lp.Replay(Path(args.replay))
    poses = {}
    for backend in ("", "onnx"):
        live = lp.new_live(Path(args.checkpoint), rp, args.run)
        if backend:
            live.use_net_backend("onnx")
        trace: list[np.ndarray] = []
        lp.replay_run(live, rp.run(args.run), on_scan=lambda k, est, dt, tr=trace: tr.extend(np.r_[e.pose, e.velocity, e.bias] for e in est))
        poses[backend or "torch"] = np.array(trace)
    a, b = poses["torch"], poses["onnx"]
    d_xy = np.hypot(a[:, 0] - b[:, 0], a[:, 1] - b[:, 1])
    print(f"{len(a)} scans; |pose torch - onnx|: xy max {d_xy.max() * 1e3:.4f} mm, yaw max {np.degrees(np.abs(a[:, 2] - b[:, 2]).max()):.6f} deg; "
          f"velocity max {np.abs(a[:, 3:5] - b[:, 3:5]).max():.2e} m/s; bias max {np.abs(a[:, 5:] - b[:, 5:]).max():.2e}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    default_ckpt = str(HERE.parent / "outputs" / "real2sim_v5_10hz" / "hybrid_submap_v4" / "best_model.pt")
    default_replay = str(HERE.parent / "outputs" / "pi4_budget" / "replay_validation.npz")
    for name, fn in (("export", cmd_export), ("check", cmd_check)):
        p = sub.add_parser(name)
        p.add_argument("--checkpoint", default=default_ckpt)
        p.add_argument("--replay", default=default_replay)
        p.add_argument("--run", type=int, default=0)
        p.set_defaults(fn=fn)
    a = ap.parse_args()
    a.fn(a)
