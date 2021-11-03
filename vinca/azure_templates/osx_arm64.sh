export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/build_osx_arm64.sh"""