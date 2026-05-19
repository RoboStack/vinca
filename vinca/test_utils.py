import json
from unittest.mock import Mock, patch

from vinca.utils import get_repodata

EMPTY_REPODATA = {"packages": {}, "packages.conda": {}}


def test_get_repodata_returns_empty_for_missing_local_repodata(tmp_path):
    assert get_repodata(str(tmp_path), "osx-arm64") == EMPTY_REPODATA


def test_get_repodata_returns_empty_for_missing_remote_repodata(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    response = Mock(status_code=404, content=b"Not Found")

    with patch("vinca.utils.requests.get", return_value=response):
        assert (
            get_repodata("https://example.com/channel", "osx-arm64") == EMPTY_REPODATA
        )


def test_get_repodata_returns_empty_for_non_json_remote_repodata(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    response = Mock(status_code=200, content=b"Not Found")
    response.raise_for_status.return_value = None

    with patch("vinca.utils.requests.get", return_value=response):
        assert (
            get_repodata("https://example.com/channel", "osx-arm64") == EMPTY_REPODATA
        )


def test_get_repodata_ignores_invalid_cached_repodata(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    url = "https://example.com/channel/osx-arm64/repodata.json"
    cache_name = "vinca_d7e1ca2423.json"
    (tmp_path / cache_name).write_text("Not Found")
    repodata = {"packages": {"pkg.tar.bz2": {"name": "pkg"}}, "packages.conda": {}}
    response = Mock(status_code=200, content=json.dumps(repodata).encode("utf-8"))
    response.raise_for_status.return_value = None

    with patch("vinca.utils.requests.get", return_value=response):
        assert get_repodata(url) == repodata
