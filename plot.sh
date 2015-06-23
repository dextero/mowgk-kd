#!/bin/bash

set -e

function die() {
    echo "$@" >&2
    exit 1
}

INPUT_FILE="$1"
PLOTS_DIR="$(readlink -f "plots")"

[[ "$INPUT_FILE" ]] || INPUT_FILE='results.txt'
[[ -f "$INPUT_FILE" ]] || die "$INPUT_FILE does not exist"
#[[ ! -d "$PLOTS_DIR" ]] || die "directory $PLOTS_DIR already exists!"

rm -rf "$PLOTS_DIR"
mkdir -p "$PLOTS_DIR"
echo "generating gnuplot script"

TMPFILE="$(mktemp)"
    cat >"$TMPFILE" <<EOF
set term png size 1024, 768
set xlabel 'required accuracy'

set style line 1 lw 2
set style line 2 lw 2

EOF

FUNCTIONS=("3d gaussian" "distance to a line" "average distance to 3 points" "Schwefel function")

for FUNC in 0 1 2 3; do
    GRAD_INPUT=$(mktemp)
    HALF_INPUT=$(mktemp)
    awk "{ if (\$1 == \"grad\" && \$2 == $FUNC) { print } }" "$INPUT_FILE" >"$GRAD_INPUT"
    awk "{ if (\$1 == \"half\" && \$2 == $FUNC) { print } }" "$INPUT_FILE" >"$HALF_INPUT"

    cat >>"$TMPFILE" <<EOF
set output '$PLOTS_DIR/build_time.$FUNC.png'
set title '${FUNCTIONS[$FUNC]}, tree build time [s]'
set ylabel 'build time [s]'
plot \\
    '$GRAD_INPUT' using 3:4 with lines title 'gradient splitter', \\
    '$HALF_INPUT' using 3:4 with lines title 'half splitter', \\

set output '$PLOTS_DIR/access_time.$FUNC.png'
set title '${FUNCTIONS[$FUNC]}, average tree access time [us]'
set ylabel 'access time [us]'
plot \\
    '$GRAD_INPUT' using 3:5 with lines title 'gradient splitter', \\
    '$HALF_INPUT' using 3:5 with lines title 'half splitter', \\

set output '$PLOTS_DIR/nodes_count.$FUNC.png'
set title '${FUNCTIONS[$FUNC]}, total number of nodes'
set ylabel '# of nodes'
plot \\
    '$GRAD_INPUT' using 3:6 with lines title 'gradient splitter', \\
    '$HALF_INPUT' using 3:6 with lines title 'half splitter', \\

set output '$PLOTS_DIR/balance_mean.$FUNC.png'
set title '${FUNCTIONS[$FUNC]}, balance (mean)'
set ylabel 'balance'
plot \\
    '$GRAD_INPUT' using 3:7 with lines title 'gradient splitter', \\
    '$HALF_INPUT' using 3:7 with lines title 'half splitter', \\

set output '$PLOTS_DIR/balance_stdev.$FUNC.png'
set title '${FUNCTIONS[$FUNC]}, tree balance (standard deviation)'
set ylabel 'balance (stdev)'
plot \\
    '$GRAD_INPUT' using 3:8 with lines title 'gradient splitter', \\
    '$HALF_INPUT' using 3:8 with lines title 'half splitter', \\

set output '$PLOTS_DIR/error_max.$FUNC.png'
set title '${FUNCTIONS[$FUNC]}, maximum error'
set ylabel 'max error'
plot \\
    '$GRAD_INPUT' using 3:9 with lines title 'gradient splitter', \\
    '$HALF_INPUT' using 3:9 with lines title 'half splitter', \\

set output '$PLOTS_DIR/error_mean.$FUNC.png'
set title '${FUNCTIONS[$FUNC]}, average error'
set ylabel 'avg error'
plot \\
    '$GRAD_INPUT' using 3:10 with lines title 'gradient splitter', \\
    '$HALF_INPUT' using 3:10 with lines title 'half splitter', \\

EOF
done

echo "plotting $TMPFILE"
gnuplot "$TMPFILE"
echo "plots saved to $PLOTS_DIR"
