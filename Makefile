# Run from repository root.
.PHONY: run delete-stats plot plot-all

run:
	bash scripts/run.sh

delete-stats:
	bash scripts/delete_stats.sh

plot:
	.venv/bin/python3 scripts/statistics.py

plot-all:
	.venv/bin/python3 scripts/statistics_all.py
