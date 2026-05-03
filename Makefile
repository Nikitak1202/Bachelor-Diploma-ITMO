SIMS ?= 4
DURATION_SEC ?= 60
PARALLEL ?= 4
BAG_FINALIZE_SEC ?= 10

RUN ?=

.PHONY: run clean stats plot plot-all

run: # run the simulation
	bash scripts/run.sh

clean: # delete all stats files
	bash scripts/delete_stats.sh


plot: # Default: latest complete bag (run_… or runN by mtime). Or: make plot RUN=2 -> logs/rosbag/run2
	.venv/bin/python3 scripts/statistics.py $(if $(RUN),--bag logs/rosbag/run$(RUN),)

plot-all: # plot statistics for all runs
	.venv/bin/python3 scripts/statistics_all.py

stats: # collect statistics for the latest run
	COLLECT_STATS_SIMS="$(SIMS)" COLLECT_STATS_PARALLEL="$(PARALLEL)" COLLECT_STATS_DURATION_SEC="$(DURATION_SEC)" COLLECT_STATS_BAG_FINALIZE_SEC="$(BAG_FINALIZE_SEC)" bash scripts/collect_stats.sh
