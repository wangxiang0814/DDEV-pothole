import json
from pathlib import Path


def test_recommended_parameters_match_completed_truth_pass_evidence():
    root = Path(__file__).resolve().parents[1]
    recommended = json.loads((root / 'configs/recommended_right_side_20261005.json').read_text(encoding='utf-8'))
    evidence = json.loads((root / 'evidence/static_fr_ii/parameter_robustness_20261005.json').read_text(encoding='utf-8'))
    trials = {trial['label']: trial for trial in evidence['trials']}
    for case in recommended['cases'].values():
        source = case['validation_source']
        trial = trials[source['set']]
        run = next(r for r in trial['runs'] if r['case'] == source['case'])
        assert run['passed'] and run['status'] == 'PASS'
        assert all(run['criteria'].values()) and all(run['front_criteria'].values())
        actual_case = trial['manifest']['cases'][source['case']]
        assert {**recommended['common'], **case['options']} == {
            **trial['manifest']['common'], **actual_case['options']}
        for key in ('requires_reference_model', 'requires_environment_reference'):
            assert bool(case.get(key)) == bool(actual_case.get(key))
