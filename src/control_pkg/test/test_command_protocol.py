from control_pkg.command_protocol import validate_voice_command


def test_supported_target_is_accepted_and_normalized():
    decision = validate_voice_command(' FETCH ', ' Coke ')

    assert decision.accepted is True
    assert decision.action == 'fetch'
    assert decision.target_class == 'coke'


def test_unknown_command_is_rejected():
    decision = validate_voice_command('unknown', 'none')

    assert decision.accepted is False
    assert 'unknown' in decision.message


def test_unsupported_target_does_not_pass_validation():
    decision = validate_voice_command('fetch', 'snack')

    assert decision.accepted is False
    assert decision.target_class == 'snack'


def test_all_planned_model_classes_are_supported():
    for target_class in ('airpod', 'coke', 'tissue', 'vaseline'):
        assert validate_voice_command('fetch', target_class).accepted is True


def test_voice_stop_is_accepted_only_without_target():
    assert validate_voice_command('stop', '').accepted is True
    assert validate_voice_command('stop', 'airpod').accepted is False
