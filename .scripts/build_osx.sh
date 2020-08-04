#!/bin/bash

set -x

echo -e "\n\nInstalling a fresh version of Miniforge."
MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download"
MINIFORGE_FILE="Miniforge3-MacOSX-x86_64.sh"
curl -L -O "${MINIFORGE_URL}/${MINIFORGE_FILE}"
/bin/bash $MINIFORGE_FILE -b

echo -e "\n\nConfiguring conda."

source ${HOME}/miniforge3/etc/profile.d/conda.sh
conda activate base

echo -e "\n\nInstalling conda-forge-ci-setup=3 and conda-build."
conda install -n base --quiet --yes conda-forge-ci-setup=3 conda-build pip boa quetz-client

# install boa
git clone https://github.com/thesnakepit/boa
cd boa
pip install -e .
cd ..

echo -e "\n\nSetting up the condarc and mangling the compiler."
setup_conda_rc ./ ./recipe ./.ci_support/${CONFIG}.yaml
mangle_compiler ./ ./recipe .ci_support/${CONFIG}.yaml

echo -e "\n\nMangling homebrew in the CI to avoid conflicts."
# /usr/bin/sudo mangle_homebrew
# /usr/bin/sudo -k

echo -e "\n\nRunning the build setup script."
source run_conda_forge_build_setup

set -e

echo -e "\n\nMaking the build clobber file and running the build."
make_build_number ./ ./recipe ./.ci_support/${CONFIG}.yaml

cd examples
vinca

boa build .

# conda build ./recipe -m ./.ci_support/${CONFIG}.yaml --clobber-file ./.ci_support/clobber_${CONFIG}.yaml

# if [[ "${UPLOAD_PACKAGES}" != "False" ]]; then
#   echo -e "\n\nUploading the packages."
#   upload_package --validate --feedstock-name="libsolv-feedstock" ./ ./recipe ./.ci_support/${CONFIG}.yaml
# fi
