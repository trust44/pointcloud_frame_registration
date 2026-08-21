import json

def test_summary_jsonl_and_report_formatting(tmp_path):
    from frame_alignment.io.registration_summary import (
        format_registration_report, read_summary_jsonl)

    summary = {
        "frame_id": "190",
        "x_offset_m": 0.1,
        "y_offset_m": 0.2,
        "correction_yaw_pitch_roll_deg": [0.3, 0.4, 0.5],
        "z_offset_m": 0.45,
        "corrected": {"count": 3822, "mean_m": 0.0692369, "p50_m": 0.0678644},
        "identity": {"count": 3822, "mean_m": 0.1869474, "p50_m": 0.1880621},
    }
    path = tmp_path / "summary.jsonl"
    path.write_text(json.dumps(summary) + "\n", encoding="utf-8")
    records = read_summary_jsonl(path)
    gt = {"manual_delta_about_lidar_origin": {
        "yaw_deg": 0.28, "pitch_deg": 0.35, "roll_deg": 0.45, "dz_m": 0.42}}

    report = format_registration_report(records["190"], gt)

    assert "yaw/deg" in report
    assert "tx/m" in report
    assert "0.1000" in report
    assert "ty/m" in report
    assert "0.2000" in report
    assert "0.3000" in report
    assert "0.2800" in report
    assert "0.0200" in report
    assert "mean/m" in report
    assert "-0.1177" in report
