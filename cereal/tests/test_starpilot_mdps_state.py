from cereal import custom


def test_starpilot_mdps_state_schema_round_trip():
  state = custom.StarPilotCarState.new_message()
  state.mdpsWarningLamp = 3
  state.mdpsLkaPlugin = 1
  state.mdpsLkaToiActive = 1
  state.mdpsLkaToiUnavailable = 0
  state.mdpsLkaToiFault = 1
  state.mdpsLkaFail = 1

  with custom.StarPilotCarState.from_bytes(state.to_bytes()) as decoded:
    assert decoded.mdpsWarningLamp == 3
    assert decoded.mdpsLkaPlugin == 1
    assert decoded.mdpsLkaToiActive == 1
    assert decoded.mdpsLkaToiUnavailable == 0
    assert decoded.mdpsLkaToiFault == 1
    assert decoded.mdpsLkaFail == 1
