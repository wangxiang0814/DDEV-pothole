import importlib.util
from pathlib import Path
import pytest


def analyzer_module():
    p=Path(__file__).resolve().parents[2]/'scripts/analyze_shallow_mpc_validation.py'
    spec=importlib.util.spec_from_file_location('validation_analyzer',p)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_incomplete_matrix_is_rejected_before_reading_any_holdout():
    module=analyzer_module()
    with pytest.raises(ValueError,match='complete'):
        module.validate_collection({'status':'RUNNING','runs':[],'total_planned':30})


def test_failed_native_worker_cannot_enter_model_validation():
    module=analyzer_module()
    with pytest.raises(ValueError,match='worker'):
        module.validate_collection({'status':'COMPLETED','runs':[{'exit_code':1}], 'total_planned':1})


def test_existing_npz_rejects_whole_analysis_before_any_output_write(tmp_path):
    module=analyzer_module()
    old=tmp_path/'prediction.npz'
    old.write_bytes(b'frozen')
    new=tmp_path/'new_summary.json'
    with pytest.raises(FileExistsError):
        module.preflight_outputs([new,old])
    assert old.read_bytes()==b'frozen'
    assert not new.exists()
