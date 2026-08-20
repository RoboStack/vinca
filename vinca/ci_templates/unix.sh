export CI=github
export GIT_BRANCH=${GITHUB_REF_NAME}
export FEEDSTOCK_NAME=${GITHUB_REPOSITORY##*/}
.scripts/build_unix.sh --target $BUILD_TARGET
