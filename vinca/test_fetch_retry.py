from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import requests

from vinca import distro as distro_module
from vinca.distro import Distro


def fake_distro():
    return SimpleNamespace(_get_auth_headers=lambda url: {})


def response(status):
    r = Mock(status_code=status)
    if status >= 400:
        r.raise_for_status.side_effect = requests.HTTPError(f"{status}", response=r)
    return r


def test_retries_transient_failures(monkeypatch):
    calls = [requests.ConnectionError("reset"), response(503), response(200)]
    get = Mock(
        side_effect=lambda *a, **k: (_ for _ in ()).throw(c)
        if isinstance(c := calls.pop(0), Exception)
        else c
    )
    monkeypatch.setattr(distro_module.requests, "get", get)
    sleep = Mock()
    monkeypatch.setattr(distro_module.time, "sleep", sleep)

    assert (
        Distro._get(fake_distro(), "https://raw.githubusercontent.com/x").status_code
        == 200
    )
    assert get.call_count == 3
    assert [c.args[0] for c in sleep.call_args_list] == [2, 4]


def test_does_not_retry_client_errors(monkeypatch):
    get = Mock(return_value=response(404))
    monkeypatch.setattr(distro_module.requests, "get", get)
    monkeypatch.setattr(distro_module.time, "sleep", Mock())

    with pytest.raises(requests.HTTPError):
        Distro._get(fake_distro(), "https://raw.githubusercontent.com/x")
    assert get.call_count == 1


def test_gives_up_after_the_last_attempt(monkeypatch):
    get = Mock(return_value=response(503))
    monkeypatch.setattr(distro_module.requests, "get", get)
    monkeypatch.setattr(distro_module.time, "sleep", Mock())

    with pytest.raises(requests.HTTPError):
        Distro._get(fake_distro(), "https://raw.githubusercontent.com/x")
    assert get.call_count == distro_module._FETCH_ATTEMPTS
