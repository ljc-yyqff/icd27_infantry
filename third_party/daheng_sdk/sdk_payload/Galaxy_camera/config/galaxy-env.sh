#!/bin/bash

if [[ ":$GENICAM_GENTL64_PATH:" != *":/usr/lib:"* ]]; then
	export GENICAM_GENTL64_PATH="${GENICAM_GENTL64_PATH:+$GENICAM_GENTL64_PATH:}/usr/lib"
fi

if [[ ":$GENICAM_GENTL32_PATH:" != *":/usr/lib:"* ]]; then
	export GENICAM_GENTL32_PATH="${GENICAM_GENTL32_PATH:+$GENICAM_GENTL32_PATH:}/usr/lib"
fi

