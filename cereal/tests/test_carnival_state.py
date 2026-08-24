from cereal import log


def test_carnival_state_schema_round_trip():
  event = log.Event.new_message()
  state = event.init("carnivalState")
  state.active = True
  state.overallConfidence = 0.9
  state.leadSource = "radar+vision"
  state.stopState = "hold"
  state.cutInCandidateCount = 2
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    assert decoded.which() == "carnivalState"
    assert decoded.carnivalState.active
    assert decoded.carnivalState.overallConfidence > 0.89
    assert decoded.carnivalState.leadSource == "radar+vision"
    assert decoded.carnivalState.stopState == "hold"
    assert decoded.carnivalState.cutInCandidateCount == 2
