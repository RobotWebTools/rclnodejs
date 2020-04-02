#!/bin/bash
#
# Copyright (c) 2017 Intel Corporation. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#Usage: npm-pack.sh

WORKDIR=`mktemp -d`
RAWMODULEDIR="rclnodejs/"
MODULEDIR="$WORKDIR/$RAWMODULEDIR"

mkdir -p $MODULEDIR
rsync -a . $MODULEDIR

cp -f scripts/npmjs-readme.md $MODULEDIR/README.md

pushd . > /dev/null
cd $WORKDIR
FILENAME=`npm pack $RAWMODULEDIR`
TARFILENAME="$WORKDIR/$FILENAME"

popd > /dev/null
mkdir -p dist
cp -f $TARFILENAME ./dist/
rm -rf $WORKDIR

echo "Generated ./dist/$FILENAME"
