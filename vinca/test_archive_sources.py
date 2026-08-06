import io
import tarfile
import zipfile

import pytest
import requests

from vinca.distro import (
    Distro,
    _read_archive_member,
    _strip_common_root,
    is_archive_url,
)
from vinca.main import _source_reference

PACKAGE_XML = "<package format='3'><name>demo_pkg</name></package>"


def _zip_bytes(files):
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for name, content in files.items():
            if name.endswith("/"):
                archive.writestr(zipfile.ZipInfo(name), b"")
            else:
                archive.writestr(name, content)
    return buffer.getvalue()


def _tar_bytes(files):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        for name, content in files.items():
            if name.endswith("/"):
                info = tarfile.TarInfo(name.rstrip("/"))
                info.type = tarfile.DIRTYPE
                archive.addfile(info)
                continue
            payload = content.encode()
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))
    return buffer.getvalue()


@pytest.mark.parametrize(
    "url, expected",
    [
        ("https://example.com/pkg-1.0.zip", True),
        ("https://example.com/pkg-1.0.tar.gz", True),
        ("https://example.com/pkg-1.0.tar.bz2", True),
        ("https://example.com/pkg.zip?token=abc", True),
        ("https://github.com/org/repo.git", False),
        ("https://gitlab.com/org/repo", False),
        (None, False),
    ],
)
def test_is_archive_url(url, expected):
    assert is_archive_url(url) is expected


def test_strip_common_root():
    assert _strip_common_root([("root/", True), ("root/a.txt", False)]) == "root"
    # A tar lists its directories without a trailing slash.
    assert _strip_common_root([("root", True), ("root/a.txt", False)]) == "root"
    # A tar created from '.' prefixes every member.
    assert (
        _strip_common_root([(".", True), ("./root", True), ("./root/a.txt", False)])
        == "root"
    )
    # Several top-level entries mean there is nothing to strip.
    assert _strip_common_root([("a/x", False), ("b/x", False)]) == ""
    # A top-level file is not a root directory.
    assert _strip_common_root([("root", False), ("root/x", False)]) == ""


@pytest.mark.parametrize("pack", [_zip_bytes, _tar_bytes])
def test_read_archive_member_strips_the_common_root(pack):
    payload = pack(
        {
            "demo-1.0/": "",
            "demo-1.0/src/demo_pkg/package.xml": PACKAGE_XML,
        }
    )
    content = _read_archive_member(
        payload=payload,
        url="https://example.com/demo-1.0.zip",
        member="src/demo_pkg/package.xml",
    )
    assert content == PACKAGE_XML


@pytest.mark.parametrize("pack", [_zip_bytes, _tar_bytes])
def test_read_archive_member_without_a_common_root(pack):
    payload = pack({"package.xml": PACKAGE_XML, "CMakeLists.txt": ""})
    content = _read_archive_member(
        payload=payload, url="https://example.com/demo-1.0.zip", member="package.xml"
    )
    assert content == PACKAGE_XML


def test_read_archive_member_of_a_dot_prefixed_tar():
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        for name in (".", "./demo-1.0", "./demo-1.0/src"):
            info = tarfile.TarInfo(name)
            info.type = tarfile.DIRTYPE
            archive.addfile(info)
        payload = PACKAGE_XML.encode()
        info = tarfile.TarInfo("./demo-1.0/src/package.xml")
        info.size = len(payload)
        archive.addfile(info, io.BytesIO(payload))
    content = _read_archive_member(
        payload=buffer.getvalue(),
        url="https://example.com/demo-1.0.tar.gz",
        member="src/package.xml",
    )
    assert content == PACKAGE_XML


def test_read_archive_member_missing_member():
    payload = _zip_bytes({"demo-1.0/": "", "demo-1.0/README.md": ""})
    with pytest.raises(KeyError):
        _read_archive_member(
            payload=payload,
            url="https://example.com/demo-1.0.zip",
            member="package.xml",
        )


def test_read_archive_member_rejects_a_directory_member():
    payload = _zip_bytes({"demo-1.0/": "", "demo-1.0/package.xml/": ""})
    with pytest.raises(KeyError):
        _read_archive_member(
            payload=payload,
            url="https://example.com/demo-1.0.zip",
            member="package.xml",
        )


def test_get_released_repo_returns_the_checksum_of_an_archive():
    distro = Distro.__new__(Distro)
    distro.snapshot = {
        "demo_pkg": {
            "url": "https://example.com/demo-1.0.zip",
            "sha256": "abc123",
            "version": "1.0",
        }
    }
    assert distro.get_released_repo("demo_pkg") == (
        "https://example.com/demo-1.0.zip",
        "abc123",
        "sha256",
    )


def test_get_released_repo_rejects_an_archive_without_a_checksum():
    distro = Distro.__new__(Distro)
    distro.snapshot = {"demo_pkg": {"url": "https://example.com/demo-1.0.zip"}}
    with pytest.raises(RuntimeError, match="no sha256 checksum"):
        distro.get_released_repo("demo_pkg")


def test_get_released_repo_still_returns_a_git_reference():
    distro = Distro.__new__(Distro)
    distro.snapshot = {
        "demo_pkg": {"url": "https://github.com/org/repo.git", "rev": "deadbeef"}
    }
    assert distro.get_released_repo("demo_pkg") == (
        "https://github.com/org/repo.git",
        "deadbeef",
        "rev",
    )


def test_package_xml_is_read_from_a_downloaded_archive(monkeypatch):
    distro = Distro.__new__(Distro)
    distro._additional_xml_cache = {}
    distro._last_archive = None
    payload = _zip_bytes(
        {"demo-1.0/": "", "demo-1.0/src/demo_pkg/package.xml": PACKAGE_XML}
    )
    downloads = []

    def fake_download(url):
        downloads.append(url)
        return payload

    monkeypatch.setattr(distro, "_download_archive_or_cached", fake_download)
    pkg_info = {
        "url": "https://example.com/demo-1.0.zip",
        "sha256": "abc123",
        "additional_folder": "src/demo_pkg",
    }
    assert distro.get_package_xml_for_additional_package(pkg_info) == PACKAGE_XML
    # A second lookup is served from the cache.
    assert distro.get_package_xml_for_additional_package(pkg_info) == PACKAGE_XML
    assert downloads == ["https://example.com/demo-1.0.zip"]


def test_auth_headers_cover_only_the_hosters_vinca_knows(monkeypatch):
    distro = Distro.__new__(Distro)
    monkeypatch.setenv("GITHUB_TOKEN", "gh-token")
    assert distro._get_auth_headers("https://raw.githubusercontent.com/o/r/f") == {
        "Authorization": "token gh-token"
    }
    # Every other host is left to requests, which resolves the ambient credentials.
    assert distro._get_auth_headers("https://artifacts.example.com/demo.zip") == {}


def test_get_forces_no_credentials_of_its_own(monkeypatch):
    """Only the wiring of _get: that it raises for status and forces no auth.

    What requests then does with the ambient netrc, proxy and CA settings is its
    own behaviour and is not re-tested here.
    """
    distro = Distro.__new__(Distro)
    monkeypatch.delenv("GITHUB_TOKEN", raising=False)
    calls = {}

    class FakeResponse:
        content = b"payload"
        text = "payload"

        def raise_for_status(self):
            calls["raised"] = True

    def fake_get(url, **kwargs):
        calls["url"] = url
        calls["kwargs"] = kwargs
        return FakeResponse()

    monkeypatch.setattr(requests, "get", fake_get)
    assert distro._get("https://artifacts.example.com/demo.zip").content == b"payload"
    assert calls["url"] == "https://artifacts.example.com/demo.zip"
    assert calls["raised"] is True
    # No auth is forced on the request, so requests applies the ambient credentials.
    assert calls["kwargs"]["headers"] == {}
    assert "auth" not in calls["kwargs"]


def test_requests_strips_authorization_when_a_redirect_changes_host():
    """The redirect behaviour vinca relies on instead of handling it itself.

    An Artifactory server answers a download with a redirect to a presigned URL on a
    separate storage host, where the original credentials must not be replayed.
    """
    session = requests.Session()
    assert session.should_strip_auth(
        "https://artifacts.example.com/demo.zip", "https://storage.example.net/signed"
    )
    assert not session.should_strip_auth(
        "https://artifacts.example.com/demo.zip", "https://artifacts.example.com/other"
    )


def test_archive_download_goes_through_get_and_is_cached(monkeypatch):
    distro = Distro.__new__(Distro)
    distro._last_archive = None
    urls = []

    class FakeResponse:
        content = b"payload"

    def fake_get(url):
        urls.append(url)
        return FakeResponse()

    monkeypatch.setattr(distro, "_get", fake_get)
    url = "https://artifacts.example.com/demo.zip"
    assert distro._download_archive_or_cached(url) == b"payload"
    assert distro._download_archive_or_cached(url) == b"payload"
    assert urls == [url]


def test_source_reference_switches_between_git_and_archive():
    assert _source_reference(
        url="https://example.com/demo-1.0.zip", ref="abc123", ref_type="sha256"
    ) == {"url": "https://example.com/demo-1.0.zip", "sha256": "abc123"}
    assert _source_reference(
        url="https://github.com/org/repo.git", ref="deadbeef", ref_type="rev"
    ) == {"git": "https://github.com/org/repo.git", "rev": "deadbeef"}
