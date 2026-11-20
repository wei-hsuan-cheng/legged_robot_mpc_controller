"""Package-wide syntax, dependency, resource, and source-coverage checks."""

import ast
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

from wb_id_parameter_contract_test import UniqueLoader
import yaml


ROOT = Path(__file__).resolve().parents[1]


class PackageHealthTest(unittest.TestCase):
    def test_python_sources_parse(self):
        for folder in ('launch', 'description', 'tests'):
            for path in (ROOT / folder).rglob('*.py'):
                with self.subTest(path=path.relative_to(ROOT)):
                    ast.parse(path.read_text(), filename=str(path))

    def test_yaml_has_no_duplicate_keys(self):
        for folder in ('config', 'src'):
            for path in (ROOT / folder).rglob('*.yaml'):
                with self.subTest(path=path.relative_to(ROOT)):
                    yaml.load(path.read_text(), Loader=UniqueLoader)

    def test_xml_resources_parse(self):
        paths = [ROOT / 'package.xml', ROOT / 'legged_robot_mpc_controller_plugin.xml']
        paths.extend(path for path in (ROOT / 'description').rglob('*')
                     if path.suffix in ('.xml', '.urdf', '.xacro'))
        for path in paths:
            with self.subTest(path=path.relative_to(ROOT)):
                ET.parse(path)

    def test_direct_dependencies_are_declared(self):
        manifest = ET.parse(ROOT / 'package.xml').getroot()
        common = {entry.text for entry in manifest.findall('depend')}
        runtime = common | {entry.text for entry in manifest.findall('exec_depend')}
        self.assertTrue({'eigen', 'boost', 'builtin_interfaces', 'tf2'} <= common)
        self.assertTrue({'controller_manager_msgs', 'python3-tk'} <= runtime)

    def test_install_excludes_generated_caches(self):
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        for name in ('__pycache__', '*.pyc', '*.pyo', '.pytest_cache', '.DS_Store'):
            self.assertIn(f'PATTERN "{name}" EXCLUDE', cmake)
        self.assertFalse((ROOT / 'cmake/Findtl-expected.cmake').exists())

    def test_cpp_sources_have_build_coverage(self):
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        for path in (ROOT / 'src').rglob('*.cpp'):
            relative = path.relative_to(ROOT).as_posix()
            if relative.startswith('src/mpc/'):
                group = relative.split('/')[2]
                self.assertIn(f'src/mpc/{group}/*.cpp', cmake)
            else:
                self.assertIn(relative, cmake)

    def test_public_headers_have_include_sites(self):
        included = set()
        for folder in ('include', 'src', 'tests'):
            for path in (ROOT / folder).rglob('*'):
                if path.suffix in ('.h', '.hpp', '.cpp'):
                    included.update(re.findall(
                        r'#\s*include\s*[<"]([^>"\n]+)[>"]', path.read_text()))
        for path in (ROOT / 'include').rglob('*'):
            if path.suffix in ('.h', '.hpp'):
                self.assertIn(path.relative_to(ROOT / 'include').as_posix(), included)


if __name__ == '__main__':
    unittest.main()
