import copy
import json
import threading
import time
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from handeye_web.core import Session, default_config, solver, validate_config
from handeye_web.franka_readonly import flange_pose, interpolate_pose
from tests.synthetic_source import DemoSource
from handeye_web.server import App, handler_class


def observe_stable(session, source):
    canvas, row, status = source.next()
    assert row, status
    now = time.time_ns()
    for n in range(8):
        current = copy.deepcopy(row)
        current["image_stamp_ns"] = now - (7-n)*100_000_000
        session.observe(canvas, current, status)
    return row


@pytest.mark.parametrize("sample_count", [10, 25])
def test_rendered_board_full_fit_and_independent_validation(tmp_path, sample_count):
    cfg = default_config()
    session = Session(tmp_path, cfg, "demo", "left")
    source = DemoSource(cfg)
    for i in range(sample_count):
        observe_stable(session, source)
        assert session.readiness()[0], session.readiness()[1]
        session.capture()
        source.advance()
        if i == 8:
            with pytest.raises(ValueError, match="至少需要 10 组"):
                session.solve()
    result = session.solve()
    assert result["quality_pass"]
    assert not result["deployment_eligible"]
    assert np.linalg.norm(np.array(result["T_base_camera"])[:3, 3]-source.x[:3, 3]) < .003
    delta = source.x[:3, :3].T @ np.array(result["T_base_camera"])[:3, :3]
    assert np.degrees(Rotation.from_matrix(delta).magnitude()) < .3
    validation = Session(tmp_path, cfg, "demo", "left", "validation", session.id)
    source.index = 40
    source.cache = None
    for _ in range(10):
        observe_stable(validation, source)
        validation.capture()
        source.advance()
    report = validation.validate(result)
    assert report["validation_pass"]
    assert not report["deployment_eligible"]
    loaded = Session.load(session.path)
    assert loaded.result == result
    assert len(loaded.data["samples"]) == sample_count
    assert len(list(session.path.glob('sample_*.png'))) == sample_count


def test_stale_duplicate_and_moving_samples_are_rejected(tmp_path):
    session = Session(tmp_path, default_config(), "demo", "left")
    source = DemoSource(default_config())
    row = observe_stable(session, source)
    session.capture()
    assert "camera" in row  # capture must not mutate the observation
    observe_stable(session, source)
    with pytest.raises(ValueError, match="姿态已采集"):
        session.capture()
    source.advance()
    observe_stable(session, source)
    session.received_at -= 2
    with pytest.raises(ValueError, match="新鲜"):
        session.capture()
    session.history.clear()
    observe_stable(session, source)
    session.history[0]["T_base_tool"][0][3] += .05
    assert not session.readiness()[0]


def test_stalled_clock_and_camera_change(tmp_path):
    session = Session(tmp_path, default_config(), "demo", "left")
    source = DemoSource(default_config())
    row = observe_stable(session, source)
    session.capture()
    source.advance()
    canvas, row, status = source.next()
    row["camera"]["k"][0] += 1
    with pytest.raises(ValueError, match="相机内参"):
        session.observe(canvas, row, status)
    row["camera"]["k"][0] -= 1
    session.observe(canvas, row, status)
    old_time = session.received_at
    for _ in range(20):
        session.observe(canvas, row, status)
    assert session.received_at == old_time
    row["image_stamp_ns"] -= 1
    session.observe(canvas, row, status)
    assert session.row is None


def test_degenerate_poses_rejected():
    a = np.eye(4)
    rows = []
    for i in range(20):
        pose = a.copy()
        pose[0, 3] = .02 * i
        rows.append({"T_base_tool": pose.tolist()})
    with pytest.raises(ValueError, match="旋转变化不足"):
        solver.excitation_check(rows)


def test_column_major_ee_offset_and_interpolation():
    f_ee = solver.transform(Rotation.from_euler('z', .2).as_matrix(), [.1, .03, .17])
    states = []
    for n in range(2):
        flange = solver.transform(Rotation.from_euler('x', .1*n).as_matrix(), [.2+.01*n, .1, .6])
        states.append({"host_stamp_ns": 100_000_000+n*10_000_000, "robot_time_ms": 10+n*10,
                       "robot_mode": "idle", "O_T_EE": (flange @ f_ee).ravel(order='F').tolist(),
                       "F_T_EE": f_ee.ravel(order='F').tolist()})
        assert np.allclose(flange_pose(states[-1]), flange)
    matrix, metadata = interpolate_pose(states, 105_000_000)
    assert np.allclose(matrix[:3, 3], [.205, .1, .6])
    assert np.allclose(Rotation.from_matrix(matrix[:3, :3]).as_rotvec(), [.05, 0, 0])
    assert metadata['timestamp_basis'] == 'host_receive_time_not_hardware_sync'
    with pytest.raises(ValueError, match="覆盖图像"):
        interpolate_pose(states, 90_000_000)
    states[-1]['robot_mode'] = 'guiding'
    with pytest.raises(ValueError, match="松开示教"):
        interpolate_pose(states, 105_000_000)
    states[-1]['robot_mode'] = 'idle'
    states[-1]['host_stamp_ns'] += 100_000_000
    with pytest.raises(ValueError, match="超过 50 ms"):
        interpolate_pose(states, 105_000_000)


def test_validation_cannot_refit_or_use_wrong_fit(tmp_path):
    cfg = default_config()
    fit = Session(tmp_path, cfg, 'live', 'left')
    with pytest.raises(ValueError, match="独立验证"):
        fit.validate({})
    validation = Session(tmp_path, cfg, 'live', 'left', 'validation', 'fit-id')
    with pytest.raises(ValueError, match="不能拟合"):
        validation.solve()
    with pytest.raises(ValueError, match="不匹配"):
        validation.validate({'session': {'id': 'other'}})
    with pytest.raises(ValueError, match="不能混用"):
        validation.validate({'session': {'id': 'fit-id', 'mode': 'demo'}})


def test_config_and_cross_side_guards():
    cfg = default_config('right')
    assert cfg['robot_ip'] == '172.16.26.4'
    assert validate_config(cfg) == cfg
    cfg['tool_frame'] = 'left_fr3_link8'
    with pytest.raises(ValueError, match='固定输出'):
        validate_config(cfg)
    cfg = default_config()
    cfg['board']['marker_length_m'] = .03
    with pytest.raises(ValueError, match='必须小于'):
        validate_config(cfg)


def test_http_origin_token_and_session_lifecycle(tmp_path):
    app = App(tmp_path)
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler_class(app))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        assert urlopen(base).status == 200
        token = json.load(urlopen(base+'/api/token'))['token']
        data = json.dumps({'action': 'new', 'side': 'right', 'config': default_config('right')}).encode()
        for headers in ({}, {'X-Handeye-Token':token,'Origin':'https://evil.example'}):
            with pytest.raises(HTTPError) as exc:
                urlopen(Request(base+'/api/action', data=data, headers=headers))
            assert exc.value.code == 403
        headers = {'X-Handeye-Token': token, 'Content-Type':'application/json'}
        assert urlopen(Request(base+'/api/action', data=data, headers=headers)).status == 200
        state = json.load(urlopen(base+'/api/state'))
        assert state['session']['mode'] == 'live'
        assert state['hardware_read_only'] is True
        assert not state['connected'] and not state['ready']
        assert 'right' in state['output']
        active_id = state['session']['id']
        offline = Session(tmp_path, default_config(), 'demo', 'left')
        requests = [
            {'action':'new', 'mode':'demo', 'side':'left'},
            {'action':'new', 'mode':'fake', 'side':'left'},
            {'action':'resume', 'id':offline.id},
            {'action':'new', 'side':'left', 'purpose':'validation', 'calibration_id':offline.id},
            {'action':'next'},
        ]
        for body in requests:
            with pytest.raises(HTTPError) as exc:
                urlopen(Request(base+'/api/action', data=json.dumps(body).encode(), headers=headers))
            assert exc.value.code == 400
            current = json.load(urlopen(base+'/api/state'))
            assert current['session']['id'] == active_id
            assert all(row['mode'] == 'live' for row in current['sessions'])
            assert offline.id not in {row['id'] for row in current['sessions']}
        assert (offline.path/'samples.json').exists()
        page = urlopen(base).read().decode()
        assert '运行模式' not in page and '下一仿真姿态' not in page
        assert '硬件只读' in page
        with pytest.raises(HTTPError) as exc:
            urlopen(Request(base+'/api/action', data=json.dumps({'action':'capture'}).encode(), headers=headers))
        assert exc.value.code == 400
        with pytest.raises(HTTPError) as exc:
            urlopen(Request(base+'/api/state', headers={'Host':'evil.example'}))
        assert exc.value.code == 403
    finally:
        server.shutdown()
        server.server_close()
        app.close()
