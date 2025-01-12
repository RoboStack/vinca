export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/build_emscripten_wasm32.sh
