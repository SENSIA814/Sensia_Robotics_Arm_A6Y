"""Offline acceptance must include the transient and the on-site observation."""
import importlib.util
import json
import math
from pathlib import Path


spec = importlib.util.spec_from_file_location('j4_analysis',
    Path(__file__).resolve().parents[3]/'tools/analyze_j4_highspeed.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def test_endpoint_keeps_ringing_that_disappears_before_last_hold_second():
    rows = [dict(t_ns=int(t*1e9), actual=math.radians(q), reference=math.pi/2,
                 j4_hardware={'effort':effort})
            for t,q,effort in [(0,0,0),(2.25,90.2,10),(2.5,89.8,-10),
                               (3.0,90,4),(3.74,90,4),(5.0,90,4)]]
    m = analysis.endpoint_metrics(rows, 2.25)
    assert math.isclose(m['position_span_deg'], .4)
    assert m['effort_span_nm'] == 20
    assert m['window_coverage_s'] >= 1.4
    assert m['reference_span_deg'] == 0


def test_operator_feedback_is_required_and_negative_feedback_wins():
    assert analysis.operator_acceptance('a', []) is None
    observations = [{'run':'a','acceptable':True},
                    {'runs':['a','b'],'acceptable':False}]
    assert analysis.operator_acceptance('a', observations) is False
    assert analysis.operator_acceptance('b', observations) is False
    assert analysis.operator_acceptance('c', observations) is None
    assert analysis.operator_acceptance('c', [{'run':'c','acceptable':True}]) is True


def test_fast_outbound_cannot_hide_slow_return(tmp_path):
    metrics = dict(encoder_peak_speed_deg_s=90., plan_error_max_deg=0., overshoot_deg=0.,
                   holds={d:dict(span_deg=0.,mean_error_deg=0.) for d in ('outbound','return')})
    report = dict(passed=True,disarm_confirmed=True,planned_peak_speed_deg_s=90.,
                  move_seconds=1.,metrics=metrics)
    path = tmp_path/'report.json'
    path.write_text(json.dumps(report))
    rows = []
    for phase,origin,velocity in [('outbound',0.,90.),('return',4.,45.)]:
        for i in range(31):
            t = i/10.
            q = min(90.,velocity*t)
            actual = math.radians(q if phase == 'outbound' else 90.-q)
            rows.append(dict(t_ns=int((origin+t)*1e9),phase=phase,reason=4,
                             actual=actual,plan=actual,reference=actual,
                             hardware={},j4_hardware={'effort':0.}))
    (tmp_path/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    criteria = dict(peak_speed_fraction=[.95,1.05],plan_error_max_deg=3.,
                    hold_last_second_span_deg=.2,hold_mean_error_deg=.2,overshoot_deg=.5)
    result = analysis.summarize(path,criteria,[{'run':tmp_path.name,'acceptable':True}])
    assert result['criteria_checks']['speed']
    assert not result['criteria_checks']['both_direction_speeds']
    assert not result['accepted']
