from smart_delivery_core.delivery_state import (
    DeliveryJournal,
    DeliveryStateMachine,
    SlotActionVerifier,
    interrupted_localization_order,
    localization_ready_for_resume,
    navigation_was_interrupted_by_localization,
)


def slot(number, status, *, enabled=True, sensor_ok=True):
    return {
        "slot": number,
        "status": status,
        "enabled": enabled,
        "sensor_ok": sensor_ok,
    }


def test_delivery_state_machine_rejects_invalid_transition():
    machine = DeliveryStateMachine()
    assert machine.transition("task_accepted") == "task_accepted"
    assert machine.transition("precheck") == "precheck"
    assert machine.transition("navigating") == "navigating"

    try:
        machine.transition("idle")
    except ValueError as exc:
        assert "不允許" in str(exc)
    else:
        raise AssertionError("invalid transition should fail")


def test_navigation_can_pause_for_localization_recovery():
    machine = DeliveryStateMachine("navigating")
    assert machine.transition("waiting_localization") == "waiting_localization"
    assert machine.transition("navigating") == "navigating"


def test_localization_resume_requires_continuous_ready_grace():
    assert not localization_ready_for_resume(True, 10.0, 12.9, 3.0)
    assert localization_ready_for_resume(True, 10.0, 13.0, 3.0)
    assert not localization_ready_for_resume(False, 10.0, 20.0, 3.0)
    assert not localization_ready_for_resume(True, None, 20.0, 3.0)


def test_cancelled_navigation_detects_localization_callback_race():
    assert navigation_was_interrupted_by_localization(True, False, 4, 4)
    assert navigation_was_interrupted_by_localization(True, True, 5, 4)
    assert not navigation_was_interrupted_by_localization(True, True, 4, 4)
    assert not navigation_was_interrupted_by_localization(False, False, 5, 4)


def test_interrupted_order_is_resumed_before_route_reordering():
    queued = [
        {"task_id": "new"},
        {"task_id": "interrupted", "_resume_state": "waiting_localization"},
    ]
    assert interrupted_localization_order(queued)["task_id"] == "interrupted"


def test_borrow_requires_expected_slot_and_restored_wrong_slot():
    baseline = [slot(1, "full"), slot(2, "low"), slot(3, "disabled", enabled=False)]
    verifier = SlotActionVerifier("borrow", 1, baseline, confirm_samples=2)

    wrong = [slot(1, "full"), slot(2, "empty"), slot(3, "disabled", enabled=False)]
    result = verifier.update(wrong)
    assert result.state == "wrong_slot"
    assert result.changed_slot == 2
    assert result.changed_slots == (2,)

    correct = [slot(1, "empty"), slot(2, "low"), slot(3, "disabled", enabled=False)]
    assert verifier.update(correct).state == "verifying_action"
    assert verifier.update(correct).confirmed


def test_return_rejects_disabled_target_and_confirms_selected_empty_slot():
    baseline = [slot(1, "full"), slot(2, "empty"), slot(3, "disabled", enabled=False)]
    verifier = SlotActionVerifier("return", 2, baseline, confirm_samples=1)
    returned = [slot(1, "full"), slot(2, "low"), slot(3, "disabled", enabled=False)]
    assert verifier.update(returned).confirmed

    try:
        SlotActionVerifier("return", 3, baseline)
    except ValueError as exc:
        assert "不可用" in str(exc)
    else:
        raise AssertionError("disabled slot should not be selected")


def test_charge_state_change_is_not_treated_as_wrong_slot():
    baseline = [slot(1, "full"), slot(2, "low"), slot(3, "disabled", enabled=False)]
    verifier = SlotActionVerifier("borrow", 1, baseline, confirm_samples=1)
    changed_charge = [
        slot(1, "empty"),
        slot(2, "full"),
        slot(3, "disabled", enabled=False),
    ]
    assert verifier.update(changed_charge).confirmed


def test_multiple_wrong_slots_are_reported_together():
    baseline = [slot(1, "full"), slot(2, "full"), slot(3, "empty")]
    verifier = SlotActionVerifier("borrow", 1, baseline, confirm_samples=1)

    result = verifier.update([slot(1, "full"), slot(2, "empty"), slot(3, "full")])

    assert result.state == "wrong_slot"
    assert result.changed_slot == 2
    assert result.changed_slots == (2, 3)
    assert "2、3號槽" in result.message


def test_delivery_journal_round_trip(tmp_path):
    journal = DeliveryJournal(tmp_path / "delivery.json")
    assert journal.load() == {}
    journal.save({"state": "result_pending", "task_id": "task-1"})
    assert journal.load()["task_id"] == "task-1"
    journal.clear()
    assert journal.load() == {}
