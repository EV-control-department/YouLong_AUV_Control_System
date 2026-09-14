from smartcar_intro.status_pub import format_status


def test_format_status() -> None:
    assert format_status(0) == "Group online, count=0"
    assert format_status(12) == "Group online, count=12"
