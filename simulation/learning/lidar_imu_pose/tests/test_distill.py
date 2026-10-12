"""The window IMU encoder of v6 and the pieces of its distillation (``distill.py``). No dataset needed."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import distill  # noqa: E402
from streaming_model import IMU_WINDOW, StreamingPoseNet, WindowImuEncoder  # noqa: E402


def _windows(lengths: torch.Tensor, samples: int = IMU_WINDOW) -> torch.Tensor:
    torch.manual_seed(0)
    imu = torch.randn(*lengths.shape, samples, 6)
    return imu * (torch.arange(samples)[None, None, :, None] < lengths[..., None, None])


def test_window_encoder_same_output_cut_or_padded():
    """Training batches are cut to the longest window of the split, the live node pads to 40: same features."""
    enc = WindowImuEncoder(32)
    lengths = torch.tensor([[3, 17], [31, 0]])
    imu = _windows(lengths)
    assert torch.equal(enc(imu, lengths), enc(imu[:, :, :31].contiguous(), lengths))


def test_window_encoder_sees_the_length():
    enc = WindowImuEncoder(32)
    imu = _windows(torch.tensor([[5]]))
    assert not torch.allclose(enc(imu, torch.tensor([[5]])), enc(imu, torch.tensor([[6]])))
    with pytest.raises(ValueError):
        enc(torch.zeros(1, 1, IMU_WINDOW + 1, 6), torch.tensor([[IMU_WINDOW + 1]]))


def test_gaussian_kl_closed_form():
    torch.manual_seed(1)
    mu_t, lv_t, mu_s, lv_s = torch.randn(4, 200)
    ref = torch.distributions.kl_divergence(torch.distributions.Normal(mu_t, torch.exp(0.5 * lv_t)), torch.distributions.Normal(mu_s, torch.exp(0.5 * lv_s)))
    assert torch.allclose(distill.gaussian_kl(mu_t, lv_t, mu_s, lv_s), ref, atol=1e-5)
    assert distill.gaussian_kl(mu_t, lv_t, mu_t, lv_t).abs().max() < 1e-6


def test_student_is_the_teacher_outside_the_encoder():
    teacher = StreamingPoseNet(64, hybrid=True, sweep_profile=True)
    student = distill.student_from(teacher)
    t, s = teacher.state_dict(), student.state_dict()
    assert all(torch.equal(v, s[k]) for k, v in t.items() if not k.startswith("imu_encoder."))
    assert student.imu_encoder_type == "window" and isinstance(student.imu_encoder, WindowImuEncoder)
    trainable = distill.freeze_all_but_encoder(student)
    assert sum(p.numel() for p in trainable) == sum(p.numel() for p in student.imu_encoder.parameters())
    assert all(p.requires_grad == n.startswith("imu_encoder.") for n, p in student.named_parameters())


def test_distill_loss_is_zero_for_the_teacher_itself():
    lanes, steps = 3, 4
    out = {"delta": torch.randn(lanes, steps, 3) * 0.01, "logvar": torch.randn(lanes, steps, 3), "velocity": torch.randn(lanes, steps, 2),
           "bias": torch.randn(lanes, steps, 3) * 1e-3}
    h = torch.rand(2 * steps, lanes, 64)
    loss, parts = distill.distill_loss(out, out, h, h, 1.0)
    assert abs(float(loss)) < 1e-6 and parts["dev_mm"] == 0.0
    moved = {**out, "delta": out["delta"] + 0.001}
    assert float(distill.distill_loss(moved, out, h, h, 1.0)[0]) > 0.0
