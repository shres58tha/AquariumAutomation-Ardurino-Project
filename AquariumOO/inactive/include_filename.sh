#!/bin/bash

if [ "$#" = "0" ]; then
	echo "at least one argument is required"
	exit 1
fi

FILE=$1
GUARD="$2$(printf "%s" $FILE | tr '.' '_' | tr '[[:lower:]]' '[[:upper:]]')"

cat <<EOF >> $FILE
#include "${FILE%%.*}.h"
EOF

