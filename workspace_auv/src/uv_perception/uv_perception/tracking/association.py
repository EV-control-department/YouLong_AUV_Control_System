"""Nearest-neighbour association helpers."""


def nearest(track_states, position, max_distance):
    best = None
    best_distance = float(max_distance)
    for track_id, state in track_states.items():
        if state.position is None:
            continue
        distance = sum((float(a) - float(b)) ** 2
                       for a, b in zip(state.position, position)) ** 0.5
        if distance < best_distance:
            best = track_id
            best_distance = distance
    return best
