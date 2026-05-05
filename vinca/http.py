import os
import urllib.parse

import requests

GITHUB_HOSTS = {"github.com", "raw.githubusercontent.com", "api.github.com"}


def _github_token():
    return (
        os.environ.get("GH_TOKEN")
        or os.environ.get("GITHUB_TOKEN")
        or os.environ.get("API_TOKEN_GITHUB")
    )


def _auth_headers(url):
    host = urllib.parse.urlparse(url).hostname or ""
    if host in GITHUB_HOSTS:
        token = _github_token()
        if token:
            return {"Authorization": f"Bearer {token}"}
    return {}


def fetch(url, *, timeout=30.0):
    resp = requests.get(url, headers=_auth_headers(url), timeout=timeout)
    if resp.status_code in (401, 403):
        host = urllib.parse.urlparse(url).hostname or ""
        hint = ""
        if host in GITHUB_HOSTS and not _github_token():
            hint = (
                " (set GH_TOKEN, GITHUB_TOKEN, or API_TOKEN_GITHUB"
                " to access private repos)"
            )
        raise RuntimeError(
            f"Auth failed for {url}: HTTP {resp.status_code}{hint}"
        )
    resp.raise_for_status()
    return resp
