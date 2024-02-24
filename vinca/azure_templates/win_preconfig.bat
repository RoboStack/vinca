set "CI=azure"
call activate base

:: 4 cores available on GHA: https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners/about-github-hosted-runners
:: CPU_COUNT is passed through conda build: https://github.com/conda/conda-build/pull/1149
set CPU_COUNT=4

set PYTHONUNBUFFERED=1

conda config --set show_channel_urls true
conda config --set auto_update_conda false
conda config --set add_pip_as_python_dependency false

call setup_x64

:: Set the conda-build working directory to a smaller path
if "%CONDA_BLD_PATH%" == "" (
    set "CONDA_BLD_PATH=C:\\bld\\"
)

:: On azure, there are libcrypto*.dll & libssl*.dll under
:: C:\\Windows\\System32, which should not be there (no vendor dlls in windows folder).
:: They would be found before the openssl libs of the conda environment, so we delete them.
if defined CI (
    DEL C:\\Windows\\System32\\libcrypto-1_1-x64.dll || (Echo Ignoring failure to delete C:\\Windows\\System32\\libcrypto-1_1-x64.dll)
    DEL C:\\Windows\\System32\\libssl-1_1-x64.dll || (Echo Ignoring failure to delete C:\\Windows\\System32\\libssl-1_1-x64.dll)
)

:: Make paths like C:\\hostedtoolcache\\windows\\Ruby\\2.5.7\\x64\\bin garbage
set "PATH=%PATH:ostedtoolcache=%"

mkdir "%CONDA%\\etc\\conda\\activate.d"

echo set "CONDA_BLD_PATH=%CONDA_BLD_PATH%"         > "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"
echo set "CPU_COUNT=%CPU_COUNT%"                  >> "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"
echo set "PYTHONUNBUFFERED=%PYTHONUNBUFFERED%"    >> "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"
echo set "PATH=%PATH%"                            >> "%CONDA%\\etc\\conda\\activate.d\\conda-forge-ci-setup-activate.bat"

conda info
conda config --show-sources
conda list --show-channel-urls
