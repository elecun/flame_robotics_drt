VENV_DIR := $(CURDIR)/venv
PYTHON := $(VENV_DIR)/bin/python3

monitor:
	$(PYTHON) ./monitor.py --config ./drt.cfg

viewer3d:
	$(PYTHON) ./python/viewer3d.py --config $(CURDIR)/python/viewer3d.cfg

controller:
	$(PYTHON) ./python/controller.py --config $(CURDIR)/python/controller.cfg

run:
	$(PYTHON) ./python/viewer3d.py --config $(CURDIR)/python/viewer3d.cfg &
	$(PYTHON) ./python/controller.py --config $(CURDIR)/python/controller.cfg