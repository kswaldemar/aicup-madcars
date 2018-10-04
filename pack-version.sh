#!/usr/bin/env bash

VER=submit-`git rev-parse --short HEAD`

rm -f ${VER}.zip
cd src
FILES=`find . -type f -exec grep -Iq . {} \; -and -print`
zip -r ../${VER}.zip ${FILES}
