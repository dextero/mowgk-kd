#!/bin/bash

set -e

function die() {
    echo "$@" >&2
    exit 1
}

INPUT_FILE="$1"
PLOTS_DIR="$2"

[[ "$INPUT_FILE" ]] || INPUT_FILE='results.txt'
[[ -f "$INPUT_FILE" ]] || die "$INPUT_FILE does not exist"
[[ "$PLOTS_DIR" ]] && PLOTS_DIR="$(readlink -f $PLOTS_DIR)"
[[ "$PLOTS_DIR" ]] || PLOTS_DIR="$(readlink -f "plots")"
#[[ ! -d "$PLOTS_DIR" ]] || die "directory $PLOTS_DIR already exists!"

rm -rf "$PLOTS_DIR"
mkdir -p "$PLOTS_DIR"
echo "generating gnuplot script"

TMPFILE="$(mktemp)"
    cat >"$TMPFILE" <<EOF
#set term png size 600, 400
set term epslatex color
set xlabel 'required accuracy'
set xrange [0:1.1] reverse

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
set output '$PLOTS_DIR/build_time_$FUNC.tex'
#set title '${FUNCTIONS[$FUNC]}, tree build time [s]'
set ylabel 'build time [s]'
set logscale y
plot \\
    '$GRAD_INPUT' using 3:4 title 'gradient splitter', \\
    '$HALF_INPUT' using 3:4 title 'half splitter'
unset logscale y

set output '$PLOTS_DIR/access_time_$FUNC.tex'
#set title '${FUNCTIONS[$FUNC]}, average tree access time [us]'
set ylabel 'access time [us]'
plot \\
    '$GRAD_INPUT' using 3:5 title 'gradient splitter', \\
    '$HALF_INPUT' using 3:5 title 'half splitter', \\

set output '$PLOTS_DIR/nodes_count_$FUNC.tex'
#set title '${FUNCTIONS[$FUNC]}, total number of nodes'
set ylabel 'number of nodes'
set logscale y
plot \\
    '$GRAD_INPUT' using 3:6 title 'gradient splitter', \\
    '$HALF_INPUT' using 3:6 title 'half splitter'
unset logscale y

set output '$PLOTS_DIR/balance_mean_$FUNC.tex'
#set title '${FUNCTIONS[$FUNC]}, balance (mean)'
set ylabel 'balance'
plot \\
    '$GRAD_INPUT' using 3:(\$8/\$7) title 'gradient splitter', \\
    '$HALF_INPUT' using 3:(\$8/\$7) title 'half splitter', \\

set output '$PLOTS_DIR/error_$FUNC.tex'
#set title '${FUNCTIONS[$FUNC]}, error'
set ylabel 'error'
plot \\
    '$GRAD_INPUT' using 3:9  title 'max gradient splitter', \\
    '$HALF_INPUT' using 3:9  title 'max half splitter', \\
    '$GRAD_INPUT' using 3:10 title 'mean gradient splitter', \\
    '$HALF_INPUT' using 3:10 title 'mean half splitter', \\

EOF
done

echo "plotting $TMPFILE"
gnuplot "$TMPFILE"
echo "plots saved to $PLOTS_DIR"
