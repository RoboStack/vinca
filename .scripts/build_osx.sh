#!/bin/bash

set -x

echo -e "\n\nInstalling a fresh version of Miniforge."
MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download"
MINIFORGE_FILE="Miniforge3-MacOSX-x86_64.sh"
curl -L -O --silent "${MINIFORGE_URL}/${MINIFORGE_FILE}"
/bin/bash $MINIFORGE_FILE -b

echo -e "\n\nConfiguring conda."

source ${HOME}/miniforge3/etc/profile.d/conda.sh
conda activate base

echo -e "\n\nInstalling conda-forge-ci-setup=3 and conda-build."
conda install -n base --quiet --yes conda-forge-ci-setup=3 conda-build pip boa quetz-client \
			  -c conda-forge/label/boa_dev -c conda-forge

# install boa from master
git clone https://github.com/thesnakepit/boa
cd boa
pip install -e .
cd ..

# echo -e "\n\nSetting up the condarc and mangling the compiler."
# # setup_conda_rc ./ ./recipe ./.ci_support/${CONFIG}.yaml
# # mangle_compiler ./ ./recipe .ci_support/${CONFIG}.yaml

# echo -e "\n\nMangling homebrew in the CI to avoid conflicts."
# # /usr/bin/sudo mangle_homebrew
# # /usr/bin/sudo -k

# echo -e "\n\nRunning the build setup script."
# # source run_conda_forge_build_setup

# set -e
conda config --append channels defaults
conda config --add channels conda-forge
conda config --add channels robostack
conda config --set channel_priority strict

export "CONDA_BLD_PATH=${FEEDSTOCK_ROOT}/build_artifacts/"

# echo -e "\n\nMaking the build clobber file and running the build."
# make_build_number ./ ./recipe ./.ci_support/${CONFIG}.yaml

conda info
conda config --show-sources
conda list --show-channel-urls

cd ${FEEDSTOCK_ROOT}
pip install -e .

cd ${FEEDSTOCK_ROOT}/examples
vinca

boa build .

anaconda -t ${ANACONDA_API_TOKEN} upload ${CONDA_BLD_PATH}/osx-64/*.tar.bz2 --force
quetz-client "${QUETZ_URL}" ${CONDA_BLD_PATH} --force

# conda build ./recipe -m ./.ci_support/${CONFIG}.yaml --clobber-file ./.ci_support/clobber_${CONFIG}.yaml

# if [[ "${UPLOAD_PACKAGES}" != "False" ]]; then
#   echo -e "\n\nUploading the packages."
#   upload_package --validate --feedstock-name="libsolv-feedstock" ./ ./recipe ./.ci_support/${CONFIG}.yaml
# fi
