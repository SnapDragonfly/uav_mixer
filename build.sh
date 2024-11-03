#!/bin/bash
DL="https://github.com/openipc/firmware/releases/download/latest"

if [[ "$1" == *"star6b0" ]]; then
	CC=cortex_a7_thumb2_hf-gcc13-musl-4_9
elif [[ "$1" == *"star6e" ]]; then
	CC=cortex_a7_thumb2_hf-gcc13-glibc-4_9
elif [[ "$1" == *"goke" ]]; then
	CC=cortex_a7_thumb2-gcc13-musl-4_9
elif [[ "$1" == *"hisi" ]]; then
	CC=cortex_a7_thumb2-gcc13-musl-4_9
elif [[ "$1" == *"x86" ]]; then
	echo "x86 build ..."
else
	echo "Usage: $0 [goke|hisi|star6b0|star6e]"
	exit 1
fi

GCC=$PWD/toolchain/$CC/bin/arm-linux-gcc

if [ ! -e toolchain/$CC ]; then
	wget -c -q --show-progress $DL/$CC.tgz -P $PWD
	mkdir -p toolchain/$CC
	tar -xf $CC.tgz -C toolchain/$CC --strip-components=1 || exit 1
	rm -f $CC.tgz
fi

if [ ! -e firmware ]; then
	git clone https://github.com/openipc/firmware --depth=1
fi

if [ "$1" = "goke" ]; then
	make -B CC=$GCC
elif [ "$1" = "hisi" ]; then
	make -B CC=$GCC
elif [ "$1" = "star6b0" ]; then
	make -B CC=$GCC
elif [ "$1" = "star6e" ]; then
	make -B CC=$GCC
elif [ "$1" = "x86" ]; then
	make
fi
