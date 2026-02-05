VENV_DIR := $(CURDIR)/venv
PYTHON := $(VENV_DIR)/bin/python3

monitor:
	$(PYTHON) ./monitor.py --config ./drt.cfg

viewer:
	$(PYTHON) ./python/viewer.py --config $(CURDIR)/python/viewer.cfg

controller:
	$(PYTHON) ./python/controller.py --config $(CURDIR)/python/controller.cfg

zproxy:
	$(PYTHON) ./python/zproxy.py --config $(CURDIR)/python/zproxy.cfg

simtool:
	$(PYTHON) ./python/simtool.py --config $(CURDIR)/python/simtool.cfg

run:
	$(PYTHON) ./python/zproxy.py --config $(CURDIR)/python/zproxy.cfg &
	$(PYTHON) ./python/viewer.py --config $(CURDIR)/python/viewer.cfg &
	$(PYTHON) ./python/simtool.py --config $(CURDIR)/python/simtool.cfg 