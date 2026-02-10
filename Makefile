VENV_DIR := $(CURDIR)/venv
PYTHON := $(VENV_DIR)/bin/python3

monitor:
	$(PYTHON) ./monitor.py --config ./drt.cfg

viewero3d:
	$(PYTHON) ./python/viewero3d.py --config $(CURDIR)/python/viewero3d.cfg

viewervedo:
	$(PYTHON) ./python/viewervedo.py --config $(CURDIR)/python/viewervedo.cfg

controller:
	$(PYTHON) ./python/controller.py --config $(CURDIR)/python/controller.cfg

zproxy:
	$(PYTHON) ./python/zproxy.py --config $(CURDIR)/python/zproxy.cfg

simtool:
	$(PYTHON) ./python/simtool.py --config $(CURDIR)/python/simtool.cfg

run:
	$(PYTHON) ./python/zproxy.py --config $(CURDIR)/python/zproxy.cfg &
	$(PYTHON) ./python/viewero3d.py --config $(CURDIR)/python/viewero3d.cfg &
	$(PYTHON) ./python/simtool.py --config $(CURDIR)/python/simtool.cfg 