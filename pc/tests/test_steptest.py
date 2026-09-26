from pathlib import Path

import numpy as np
import pytest

from robotarm.analysis import steptest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def data():
    return steptest.load_step_csv(REPO / "output.txt")


def test_load_step_csv(data):
    assert len(data.t_s) == 1000
    assert data.duty.max() == pytest.approx(1.0)
    assert np.argmax(data.duty > 0) == 401
    assert data.pos[-1] == 1010


def test_committed_identification_reproduces_the_recording(data):
    params = steptest.load_identified(REPO / "config" / "bench_identified.yaml")
    sim = steptest.simulate(params, data)
    final = data.pos[-1]
    assert abs(sim[-1] - final) / final < 0.05                     # end position within 5 %
    assert np.sqrt(np.mean((sim - data.pos) ** 2)) / final < 0.05   # whole trace within 5 % RMS
    # coast-down (brake + friction) must also match: position gained after switch-off
    off = 801
    assert abs((sim[-1] - sim[off]) - (data.pos[-1] - data.pos[off])) < 30
