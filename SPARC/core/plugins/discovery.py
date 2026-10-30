"""Discover concrete plugin classes from the category directories."""
from __future__ import annotations

import importlib.util
import ast
import inspect
import sys
from pathlib import Path
from typing import Any

from plugins.base.optimizerbase import OptimizerBase
from plugins.base.plannerbase import PlannerBase
from plugins.base.posefinderbase import PoseFinderBase


BASES = {
    "pose_finder": PoseFinderBase,
    "path_planner": PlannerBase,
    "optimizer": OptimizerBase,
}


class PluginLoader:
    """Lazy loader that lets the UI list plugins with optional dependencies."""
    def __init__(self, path: Path, module_name: str, class_name: str | None, base_class: type):
        self.path = path
        self.module_name = module_name
        self.class_name = class_name
        self.base_class = base_class

    def load(self):
        spec = importlib.util.spec_from_file_location(self.module_name, self.path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load plugin {self.path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[self.module_name] = module
        spec.loader.exec_module(module)
        if self.class_name:
            return getattr(module, self.class_name)
        for _, candidate in inspect.getmembers(module, inspect.isclass):
            if candidate.__module__ == self.module_name and issubclass(candidate, self.base_class) and candidate is not self.base_class:
                return candidate
        raise ImportError(f"No {self.base_class.__name__} implementation in {self.path}")

    def __call__(self, *args, **kwargs):
        return self.load()(*args, **kwargs)


def discover_plugins(plugin_root: Path) -> dict[str, dict[str, Any]]:
    """Return category -> display name -> implementation for concrete classes."""
    discovered: dict[str, dict[str, Any]] = {}
    for category, base_class in BASES.items():
        category_path = plugin_root / category
        entries: dict[str, Any] = {}
        if category_path.is_dir():
            for source in sorted(category_path.glob("*.py")):
                if source.name.startswith("_"):
                    continue
                module_name = f"sparc_plugin_{category}_{source.stem}"
                spec = importlib.util.spec_from_file_location(module_name, source)
                if spec is None or spec.loader is None:
                    continue
                module = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = module
                try:
                    spec.loader.exec_module(module)
                except Exception as error:
                    sys.modules.pop(module_name, None)
                    tree = ast.parse(source.read_text(encoding="utf-8"))
                    base_names = {base_class.__name__}
                    found_class = False
                    for node in tree.body:
                        if isinstance(node, ast.ClassDef) and any(
                            isinstance(base, ast.Name) and base.id in base_names
                            or isinstance(base, ast.Attribute) and base.attr in base_names
                            for base in node.bases
                        ):
                            entries[source.stem] = PluginLoader(source, module_name, node.name, base_class)
                            found_class = True
                    if not found_class and any(isinstance(node, ast.ClassDef) for node in tree.body):
                        entries[source.stem] = PluginLoader(source, module_name, None, base_class)
                    print(f"Plugin {source.stem} listed; dependency load deferred ({error})")
                    continue
                for _, implementation in inspect.getmembers(module, inspect.isclass):
                    if implementation.__module__ != module_name or implementation is base_class:
                        continue
                    if issubclass(implementation, base_class) and not inspect.isabstract(implementation):
                        entries[source.stem] = implementation
        discovered[category] = entries
    return discovered
