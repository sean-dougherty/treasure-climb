NAME=treasure-climb

${NAME}:
	dasm ${NAME}.asm ${DASM_ATARI_INCLUDE} -l${NAME}.lst -s${NAME}.sym -f3 -o${NAME}.bin
	@if grep error ${NAME}.lst 2>/dev/null > errors; then head -n 10 errors; exit 1; fi
