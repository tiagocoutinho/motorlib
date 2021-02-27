import pytest

from motorlib import JogTrajectory


@pytest.mark.parametrize("pi", [0, 101, -101], ids=['pi0', 'pi+', 'pi-'])
@pytest.mark.parametrize("v", [0.1, 10, 2500, -10, -400], ids=['v_low', 'v+', 'v++', 'v-', 'v--'])
@pytest.mark.parametrize("accel", [0.01, 2, 20], ids=['a_low', 'a+', 'a++'])
@pytest.mark.parametrize("vi", [0, 0.2, 5, 500, -20, -600], ids=['vi0', 'vi_low', 'vi+', 'vi++', 'vi-', 'vi--'])
@pytest.mark.parametrize("t", [0.01, 0.25, 10, 1000, 900000], ids=["t_short", "t_min", "t", "t+", "t++"])
def test_jog(pi, v, accel, vi, t):
    ti = 0

    if v < vi:
        accel = -accel
    positive = v >= 0

    accel_time = (v - vi) / accel
    accel_dp = vi * accel_time + 0.5 * accel * accel_time ** 2

    jog = JogTrajectory(pi, v, accel, vi, ti)
    assert jog.pi == pi
    assert jog.ti == ti
    assert jog.vi == vi
    assert jog.vel == v
    assert jog.accel == accel
    assert jog.accel_time == pytest.approx(accel_time)
    assert jog.accel_dp == pytest.approx(accel_dp)
    assert jog.positive == positive
    assert jog.ta == pytest.approx(ti + accel_time)
    assert jog.pa == pytest.approx(pi + accel_dp)

    if t < (ti + accel_time):
        dt = t - ti
        p = pi + vi * dt + 0.5 * accel * dt ** 2
    else:
        dt = t - ti
        p = pi + vi * accel_time + 0.5 * accel * accel_time ** 2 + v * (dt - accel_time)
    assert jog.position(t) == pytest.approx(p)
